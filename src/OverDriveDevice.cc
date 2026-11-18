#include "OverDriveDevice.hh"

#include "MSXCliComm.hh"
#include "MSXException.hh"
#include "strCat.hh"

#include <array>
#include <cerrno>
#include <string_view>

#ifndef _WIN32
#include <netdb.h>
#include <netinet/tcp.h>
#include <sys/select.h>
#include <sys/time.h>
#endif

namespace openmsx {

namespace {

// After a failed connect/transfer, wait this long before trying again, so the
// emulation doesn't stall on every single I/O access while the server is away.
constexpr std::chrono::milliseconds RETRY_DELAY{1000};

#ifdef _WIN32
constexpr int SEND_FLAGS = 0;
#elif defined(MSG_NOSIGNAL)
constexpr int SEND_FLAGS = MSG_NOSIGNAL; // no SIGPIPE when the server went away
#else
constexpr int SEND_FLAGS = 0;
#endif

OverDriveDevice::Protocol parseProtocol(std::string_view s)
{
	if (s == "tcp") return OverDriveDevice::Protocol::TCP;
	if (s == "udp") return OverDriveDevice::Protocol::UDP;
	throw MSXException("OverDrive: unknown protocol \"", s, "\" (use tcp or udp)");
}

void setBlocking(SOCKET s, bool blocking)
{
#ifdef _WIN32
	u_long mode = blocking ? 0 : 1;
	ioctlsocket(s, FIONBIO, &mode);
#else
	int flags = fcntl(s, F_GETFL, 0);
	if (flags < 0) return;
	fcntl(s, F_SETFL, blocking ? (flags & ~O_NONBLOCK) : (flags | O_NONBLOCK));
#endif
}

timeval toTimeval(int ms)
{
	timeval tv{};
	tv.tv_sec = static_cast<decltype(tv.tv_sec)>(ms / 1000);
	tv.tv_usec = static_cast<decltype(tv.tv_usec)>((ms % 1000) * 1000);
	return tv;
}

// true: readable (or error/closed) within 'ms' milliseconds
bool waitReadable(SOCKET s, int ms)
{
	fd_set fds;
	FD_ZERO(&fds);
	FD_SET(s, &fds);
	auto tv = toTimeval(ms);
	return select(static_cast<int>(s) + 1, &fds, nullptr, nullptr, &tv) > 0;
}

// Connect with a timeout. On success the socket is left in blocking mode.
bool connectWithTimeout(SOCKET s, const sockaddr* addr, size_t addrLen, int ms)
{
	setBlocking(s, false);
	int r = connect(s, addr, static_cast<int>(addrLen));
	if (r != 0) {
#ifdef _WIN32
		bool inProgress = WSAGetLastError() == WSAEWOULDBLOCK;
#else
		bool inProgress = errno == EINPROGRESS;
#endif
		if (!inProgress) return false;

		fd_set wfds;
		fd_set efds;
		FD_ZERO(&wfds);
		FD_ZERO(&efds);
		FD_SET(s, &wfds);
		FD_SET(s, &efds);
		auto tv = toTimeval(ms);
		if (select(static_cast<int>(s) + 1, nullptr, &wfds, &efds, &tv) <= 0) {
			return false; // timeout or select error
		}
		int err = 0;
#ifdef _WIN32
		int errLen = sizeof(err);
		getsockopt(s, SOL_SOCKET, SO_ERROR, reinterpret_cast<char*>(&err), &errLen);
#else
		::socklen_t errLen = sizeof(err);
		getsockopt(s, SOL_SOCKET, SO_ERROR, &err, &errLen);
#endif
		if (err != 0) return false;
	}
	setBlocking(s, true);
	return true;
}

ptrdiff_t rawSend(SOCKET s, const char* buf, size_t len)
{
#ifdef _WIN32
	return send(s, buf, static_cast<int>(len), SEND_FLAGS);
#else
	return send(s, buf, len, SEND_FLAGS);
#endif
}

ptrdiff_t rawRecv(SOCKET s, char* buf, size_t len)
{
#ifdef _WIN32
	return recv(s, buf, static_cast<int>(len), 0);
#else
	return recv(s, buf, len, 0);
#endif
}

} // namespace

OverDriveDevice::OverDriveDevice(const DeviceConfig& config)
	: MSXDevice(config)
	, protocolSetting(
		getCommandController(), tmpStrCat(getName(), "_protocol"),
		"transport used to reach the OverDrive server: tcp or udp",
		parseProtocol(config.getChildData("protocol", "tcp")),
		EnumSetting<Protocol>::Map{
			{"tcp", Protocol::TCP},
			{"udp", Protocol::UDP}})
	, hostSetting(
		getCommandController(), tmpStrCat(getName(), "_host"),
		"host name or IP address of the OverDrive server",
		config.getChildData("host", "127.0.0.1"))
	, portSetting(
		getCommandController(), tmpStrCat(getName(), "_port"),
		"TCP/UDP port of the OverDrive server",
		config.getChildDataAsInt("port", 5000), 1, 65535)
	, timeoutSetting(
		getCommandController(), tmpStrCat(getName(), "_timeout"),
		"milliseconds to wait for the server to answer an IN",
		config.getChildDataAsInt("timeout", 200), 1, 10000)
{
}

OverDriveDevice::~OverDriveDevice()
{
	disconnect();
}

OverDriveDevice::Endpoint OverDriveDevice::currentEndpoint() const
{
	return {.protocol = protocolSetting.getEnum(),
	        .host = std::string(hostSetting.getString()),
	        .port = portSetting.getInt()};
}

void OverDriveDevice::disconnect()
{
	if (sock != OPENMSX_INVALID_SOCKET) {
		sock_close(sock);
		sock = OPENMSX_INVALID_SOCKET;
	}
}

// Something went wrong: don't try again for a while.
void OverDriveDevice::fail(const char* reason)
{
	if (connected.protocol == Protocol::TCP) {
		disconnect(); // a stream that lost sync is useless
	}
	retryAt = Clock::now() + RETRY_DELAY;
	if (!warned) {
		warned = true;
		getCliComm().printWarning("OverDrive: ", reason, " (",
		                          connected.host, ":", connected.port, ")");
	}
}

bool OverDriveDevice::ensureConnected()
{
	auto wanted = currentEndpoint();
	if (sock != OPENMSX_INVALID_SOCKET && !(wanted == connected)) {
		disconnect(); // settings changed: reconnect
		retryAt = {};
	}
	if (Clock::now() < retryAt) return false;
	if (sock != OPENMSX_INVALID_SOCKET) return true;

	connected = wanted;
	const bool tcp = wanted.protocol == Protocol::TCP;

	addrinfo hints{};
	hints.ai_family = AF_INET;
	hints.ai_socktype = tcp ? SOCK_STREAM : SOCK_DGRAM;
	addrinfo* res = nullptr;
	const std::string portStr = std::to_string(wanted.port);
	if (getaddrinfo(wanted.host.c_str(), portStr.c_str(),
	                &hints, &res) != 0 || !res) {
		fail("cannot resolve server address");
		return false;
	}

	SOCKET s = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
	bool ok = false;
	if (s != OPENMSX_INVALID_SOCKET) {
		ok = connectWithTimeout(s, res->ai_addr, res->ai_addrlen,
		                        timeoutSetting.getInt());
		if (!ok) {
			sock_close(s);
			s = OPENMSX_INVALID_SOCKET;
		}
	}
	freeaddrinfo(res);
	if (!ok) {
		fail(tcp ? "cannot connect to server" : "cannot open UDP socket");
		return false;
	}

	if (tcp) { // send every frame immediately, no Nagle delay
		int flag = 1;
		setsockopt(s, IPPROTO_TCP, TCP_NODELAY,
		           reinterpret_cast<const char*>(&flag), static_cast<int>(sizeof(flag)));
	}
	sock = s;
	warned = false;
	return true;
}

bool OverDriveDevice::sendFrame(uint8_t port, char command, uint8_t value)
{
	const std::array<char, 3> frame{static_cast<char>(port), command,
	                                static_cast<char>(value)};
	size_t sent = 0;
	while (sent < frame.size()) {
		auto n = rawSend(sock, frame.data() + sent, frame.size() - sent);
		if (n <= 0) return false;
		sent += static_cast<size_t>(n);
	}
	return true;
}

bool OverDriveDevice::recvByte(byte& result)
{
	if (!waitReadable(sock, timeoutSetting.getInt())) return false;
	char c = 0;
	if (rawRecv(sock, &c, 1) != 1) return false;
	result = static_cast<byte>(c);
	return true;
}

// UDP: throw away late replies of earlier (timed out) requests.
void OverDriveDevice::drainDatagrams()
{
	char c = 0;
	while (waitReadable(sock, 0)) {
		if (rawRecv(sock, &c, 1) <= 0) break;
	}
}

byte OverDriveDevice::readIO(uint16_t port, EmuTime /*time*/)
{
	if (!ensureConnected()) return 0xFF;
	if (connected.protocol == Protocol::UDP) drainDatagrams();

	byte result = 0xFF;
	if (!sendFrame(static_cast<uint8_t>(port & 0xFF), 'R', 0x00) ||
	    !recvByte(result)) {
		fail("no answer from server");
		return 0xFF;
	}
	return result;
}

byte OverDriveDevice::peekIO(uint16_t /*port*/, EmuTime /*time*/) const
{
	return 0xFF; // reading has side effects on the server: never peek
}

void OverDriveDevice::writeIO(uint16_t port, byte value, EmuTime /*time*/)
{
	if (!ensureConnected()) return;
	if (!sendFrame(static_cast<uint8_t>(port & 0xFF), 'W', value)) {
		fail("cannot send to server");
	}
}

} // namespace openmsx
