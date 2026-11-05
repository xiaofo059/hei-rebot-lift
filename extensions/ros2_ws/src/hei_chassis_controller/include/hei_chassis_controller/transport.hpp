#pragma once
#include "hei_chassis_controller/protocol.hpp"
#include <string>

namespace hei_chassis_controller
{
class SerialTransport
{
public:
  ~SerialTransport() {close();}
  void open(const std::string & port, int baud);
  void close();
  void send(const Frame & frame);
  void receive(PacketParser & parser, const std::function<void(const Packet &)> & consume);
private:
  int fd_{-1};
  bool exclusive_{false};
};
}  // namespace hei_chassis_controller
