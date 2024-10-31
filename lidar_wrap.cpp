#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "sl_lidar.h" 
#include "sl_lidar_driver.h"
#include <cmath>
#include <iostream>
#include <thread> // Include for std::this_thread::sleep_for

namespace py = pybind11;

class LidarWrapper {
public:
    LidarWrapper(const std::string &port = "/dev/ttyUSB0", sl_u32 baudrate = 115200)
        : drv(nullptr), channel(nullptr), port(port), baudrate(baudrate), current_measured_data(360, 0.0) {
        
        if (!initialize()) {
            throw std::runtime_error("Couldn't connect to LIDAR. Check connection, port, and baudrate.");
        }
        startScan();
    }

    ~LidarWrapper() {
        stopScan();
        delete drv;
    }

    std::vector<float> getScanData() {
        sl_lidar_response_measurement_node_hq_t nodes[8192];
        size_t count = sizeof(nodes) / sizeof(nodes[0]);

        if (SL_IS_OK(drv->grabScanDataHq(nodes, count))) {
            drv->ascendScanData(nodes, count);
            for (int pos = 0; pos < (int)count; ++pos) {
                float angle = (nodes[pos].angle_z_q14 * 90.0f) / 16384.0f;  // Angle in degrees
                float distance = nodes[pos].dist_mm_q2 / 4.0f;              // Distance in mm
                current_measured_data[std::round(angle)] = distance / 1000.0f; // Store distance in meters
            }
        }
        return current_measured_data;
    }

    bool initialize() {
        sl_lidar_response_device_info_t devinfo;
        for (int attempt = 0; attempt < 5; ++attempt) {
            createChannelAndConnect();
            if (drv && SL_IS_OK(drv->getDeviceInfo(devinfo) && checkLidarHealth())) {
                std::cout << "Connected to SLAMTEC LIDAR\n";
                return true;
            }
            std::cerr << "Device not responding, attempting again...\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(500)); // Delay before retrying
        }
        return false;
    }

    void startScan() {
        if (drv) {
            drv->setMotorSpeed();
            drv->startScan(0, 1);
        }
    }

    void stopScan() {
        if (drv) {
            drv->stop();
            drv->setMotorSpeed(0);
        }
    }

    bool checkLidarHealth() {
        sl_lidar_response_device_health_t healthinfo;
        if (SL_IS_OK(drv->getHealth(healthinfo)) && healthinfo.status != (SL_LIDAR_STATUS_ERROR)) {
            return true;
        }
        std::cerr << "LIDAR health check failed.\n";
        return false;
    }

private:
    sl::ILidarDriver *drv;
    sl::IChannel *channel;
    const std::string port;
    const sl_u32 baudrate;
    std::vector<float> current_measured_data;

    void createChannelAndConnect() {
        drv = *sl::createLidarDriver();
        if (drv) {
            channel = *sl::createSerialPortChannel(port.c_str(), baudrate);
            drv->connect(channel);
        }
    }
};

// Module definition with documentation
PYBIND11_MODULE(rplidar, m) {
    m.doc() = "SLAMTEC LIDAR Python bindings"; // Optional module docstring

    // Expose the LidarWrapper class to Python
    py::class_<LidarWrapper>(m, "LidarWrapper", "Wrapper class for SLAMTEC LIDAR.")
        .def(py::init<const std::string&, sl_u32>(),
             py::arg("port") = "/dev/ttyUSB0",
             py::arg("baudrate") = 1000000,
             "Initialize the LidarWrapper instance.\n\n"
             "Args:\n"
             "    port (str): The serial port to which the LIDAR is connected (default: '/dev/ttyUSB0').\n"
             "    baudrate (int): The baud rate for the serial communication (default: 1000000).")
        .def("initialize", &LidarWrapper::initialize, 
             "Initialize the Lidar.\n\n"
             "Returns:\n"
             "    bool: True if the Lidar is successfully initialized, False otherwise.")
        .def("get_scan_data", &LidarWrapper::getScanData, 
             "Get scan data from the Lidar.\n\n"
             "Returns:\n"
             "    list of float: 360 float list (one per degree) of distance measurements in meters.")
        .def("start_scan", &LidarWrapper::startScan, 
             "Start scanning with the Lidar.")
        .def("stop_scan", &LidarWrapper::stopScan, 
             "Stop scanning with the Lidar.");
}
