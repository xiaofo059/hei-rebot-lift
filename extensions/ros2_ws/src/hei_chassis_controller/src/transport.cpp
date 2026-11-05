#include "hei_chassis_controller/transport.hpp"
#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <stdexcept>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

namespace hei_chassis_controller
{
void SerialTransport::open(const std::string & port, int baud)
{
  if (fd_ >= 0) {throw std::runtime_error("Serial transport is already open");}
  if (baud != 921600 && baud != 115200) {throw std::runtime_error("Supported baud: 921600 or 115200");}
  fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) {throw std::runtime_error("Open " + port + ": " + std::strerror(errno));}
  try {
    if (flock(fd_, LOCK_EX | LOCK_NB) < 0 || ioctl(fd_, TIOCEXCL) < 0) {
      throw std::runtime_error("Cannot exclusively claim chassis serial port");
    }
    exclusive_ = true;
    termios tty{};
    if (tcgetattr(fd_, &tty) < 0) {throw std::runtime_error("tcgetattr failed");}
    cfmakeraw(&tty); tty.c_cflag |= CLOCAL | CREAD; tty.c_cflag &= ~CRTSCTS;
    cfsetispeed(&tty, baud == 921600 ? B921600 : B115200);
    cfsetospeed(&tty, baud == 921600 ? B921600 : B115200);
    tty.c_cc[VMIN] = 0; tty.c_cc[VTIME] = 0;
    if (tcsetattr(fd_, TCSANOW, &tty) < 0) {throw std::runtime_error("tcsetattr failed");}
    tcflush(fd_, TCIOFLUSH);
  } catch (...) {close(); throw;}
}
void SerialTransport::close()
{
  if (fd_ >= 0) {if (exclusive_) {ioctl(fd_, TIOCNXCL);} ::close(fd_); fd_ = -1; exclusive_ = false;}
}
void SerialTransport::send(const Frame & frame)
{
  size_t offset = 0;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(5);
  while (offset < frame.size()) {
    const auto n = ::write(fd_, frame.data() + offset, frame.size() - offset);
    if (n > 0) {offset += size_t(n); continue;}
    if (n < 0 && errno != EAGAIN && errno != EINTR) {throw std::runtime_error("U2CAN write failed");}
    if (std::chrono::steady_clock::now() >= deadline) {throw std::runtime_error("U2CAN write timeout");}
    pollfd descriptor{fd_, POLLOUT, 0}; poll(&descriptor, 1, 1);
  }
}
void SerialTransport::receive(PacketParser & parser,
  const std::function<void(const Packet &)> & consume)
{
  uint8_t buffer[512];
  // Bound work even if a noisy adapter continuously sends bytes.
  for (int i = 0; i < 8; ++i) {
    const auto n = ::read(fd_, buffer, sizeof(buffer));
    if (n > 0) {parser.feed(buffer, size_t(n), consume); continue;}
    if (n < 0 && errno != EAGAIN && errno != EINTR) {throw std::runtime_error("U2CAN read failed");}
    break;
  }
}
}  // namespace hei_chassis_controller
