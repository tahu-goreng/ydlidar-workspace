#include "CYdLidar.h"
#include <iostream>
#include <memory>
using namespace ydlidar;
using namespace std;

int main() {
    cout << "=== Manual YDLIDAR X4 Pro Test (C++11 safe) ===" << endl;
    CYdLidar laser;

    // Parameter setup
    string port = "/dev/ttyUSB0";
    int baudrate = 230400;   // coba juga 115200 jika gagal
    int lidarType = 4;
    int deviceType = YDLIDAR_TYPE_SERIAL;
    float scanFreq = 7.0f;
    int sampleRate = 4;
    bool singleChannel = false;
    bool autoReconnect = true;

    laser.setlidaropt(LidarPropSerialPort, port.c_str(), port.size());
    laser.setlidaropt(LidarPropSerialBaudrate, &baudrate, sizeof(int));
    laser.setlidaropt(LidarPropLidarType, &lidarType, sizeof(int));
    laser.setlidaropt(LidarPropDeviceType, &deviceType, sizeof(int));
    laser.setlidaropt(LidarPropScanFrequency, &scanFreq, sizeof(float));
    laser.setlidaropt(LidarPropSampleRate, &sampleRate, sizeof(int));
    laser.setlidaropt(LidarPropSingleChannel, &singleChannel, sizeof(bool));
    laser.setlidaropt(LidarPropAutoReconnect, &autoReconnect, sizeof(bool));

    cout << "[INFO] Initializing lidar..." << endl;
    if (laser.initialize()) {
        cout << "[OK] Lidar initialized successfully." << endl;
        if (laser.turnOn()) {
            cout << "[OK] Scanning started. Reading data..." << endl;
            LaserScan scan;
            for (int i = 0; i < 10; i++) {
                if (laser.doProcessSimple(scan)) {
                    cout << "Frame " << i+1 << ": " << scan.points.size()
                         << " points. Angle range: "
                         << scan.config.min_angle << "–" << scan.config.max_angle << endl;
                } else {
                    cout << "[WARN] No scan data received." << endl;
                }
            }
            laser.turnOff();
        } else {
            cout << "[ERROR] Failed to start scanning." << endl;
        }
    } else {
        cout << "[ERROR] Failed to initialize lidar." << endl;
    }

    laser.disconnecting();
    return 0;
}

