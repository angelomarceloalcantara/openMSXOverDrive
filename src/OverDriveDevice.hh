#ifndef OVERDRIVEDEVICE_HH
#define OVERDRIVEDEVICE_HH

#include "EnumSetting.hh"
#include "IntegerSetting.hh"
#include "MSXDevice.hh"
#include "Socket.hh"
#include "StringSetting.hh"

#include <chrono>
#include <cstdint>
#include <string>

namespace openmsx {

/** OverDrive: an I/O device that forwards every IN/OUT on its 4 ports to an
  * external "virtual device" server (e.g. a Python script) over TCP or UDP.
  *
  * Ports (relative to the base port given in the XML <io> element):
  *   base+0  device
  *   base+1  subdevice
  *   base+2  data
  *   base+3  command
  *
  * Wire format, always 3 bytes:  [I/O port][ 'R' | 'W' ][ value ]
  *   OUT: port, 'W', byte written   -> no reply
  *   IN : port, 'R', 0x00           -> the server replies with 1 byte
  */
class OverDriveDevice final : public MSXDevice
{
public:
	enum class Protocol : uint8_t { TCP, UDP };

	explicit OverDriveDevice(const DeviceConfig& config);
	~OverDriveDevice() override;

	byte readIO(uint16_t port, EmuTime time) override;
	byte peekIO(uint16_t port, EmuTime time) const override;
	void writeIO(uint16_t port, byte value, EmuTime time) override;

private:
	using Clock = std::chrono::steady_clock;

	struct Endpoint {
		Protocol protocol = Protocol::TCP;
		std::string host;
		int port = 0;
		bool operator==(const Endpoint&) const = default;
	};

	[[nodiscard]] Endpoint currentEndpoint() const;
	[[nodiscard]] bool ensureConnected();
	void disconnect();
	void fail(const char* reason);
	[[nodiscard]] bool sendFrame(uint8_t port, char command, uint8_t value);
	[[nodiscard]] bool recvByte(byte& result);
	void drainDatagrams();

private:
	SocketActivator socketActivator; // initializes winsock on Windows
	EnumSetting<Protocol> protocolSetting;
	StringSetting hostSetting;
	IntegerSetting portSetting;
	IntegerSetting timeoutSetting;

	SOCKET sock = OPENMSX_INVALID_SOCKET;
	Endpoint connected; // endpoint 'sock' is connected to
	Clock::time_point retryAt{}; // after a failure: no network I/O before this moment
	bool warned = false; // already warned about the current failure?
};

} // namespace openmsx

#endif
