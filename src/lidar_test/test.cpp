#include "CYdLidar.h"
#include <iostream>

using namespace ydlidar;

int main() {
  CYdLidar lidar;
  std::string port = "/dev/ttyUSB0";
  int baudrate = 115200; // ubah ke 128000 kalau masih gagal
  lidar.setlidaropt(LidarPropSerialPort, port.c_str(), port.size());
  lidar.setlidaropt(LidarPropSerialBaudrate, &baudrate, sizeof(int));
  lidar.initialize();

  if (lidar.turnOn()) {
      std::cout << "✅ LiDAR running correctly!" << std::endl;
  } else {
      std::cout << "❌ get Device Information Error" << std::endl;
  }

  lidar.turnOff();
  lidar.disconnecting();
  return 0;
}
