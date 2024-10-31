#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "sl_lidar.h" 
#include "sl_lidar_driver.h"
#include <cmath>

using namespace sl;
namespace py = pybind11;

class LidarWrapper {
public:
    const sl_u32 baudrate;
    const std::string port;
    LidarWrapper(const std::string &port = "/dev/ttyUSB0", sl_u32 baudrate = 1000000) 
        : drv(nullptr), channel(nullptr), ctrl_c_pressed(false), port(port), baudrate(baudrate) {

        if (initialize()) {
            startScan();
        } else {
            printf("Couldn't connect, check connection, port and baudrate");
        }
    }
    ~LidarWrapper() {
        if (drv) {
            stopScan();
            delete drv;
            drv = nullptr;
        }
    }

    void createChannelAndConnect(){
        drv = *createLidarDriver();
            if (drv) {
                channel = *createSerialPortChannel(port.c_str(), baudrate);
                drv->connect(channel);
            } 
    }

    bool initialize() {
        int counter = 0;
        while (counter++ < 5 && (!drv)) {
            sl_lidar_response_device_info_t devinfo;
            createChannelAndConnect();
            if ((drv) && SL_IS_OK(drv->getDeviceInfo(devinfo)) && checkLidarHealth()) {
                printf("Connected to SLAMTEC LIDAR\n");
                return true;
            }
            printf("Device not responding attempting again...");
        }

        return false;
    }

    std::vector<float> getScanData() {
        sl_lidar_response_measurement_node_hq_t nodes[8192];
        size_t count = sizeof(nodes) / sizeof(nodes[0]);

        if (SL_IS_OK(drv->grabScanDataHq(nodes, count))) {
            drv->ascendScanData(nodes, count);
            for (int pos = 0; pos < (int)count; ++pos) {                    // Documentation from Slamtec
                float angle = (nodes[pos].angle_z_q14 * 90.0f) / 16384.0f;  //Angle in degrees
                float distance = nodes[pos].dist_mm_q2 / 4.0f;              //Distance in mm
                int quality = nodes[pos].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;
                current_measured_data[(int)std::round(angle)] = distance / 1000.0f; //Distance in m
            }
        }
        return current_measured_data;
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

private:
    ILidarDriver *drv;
    IChannel *channel;
    std::vector<float> current_measured_data = std::vector<float>(360, 0.0);

    bool ctrl_c_pressed;
    bool checkLidarHealth() {
        sl_lidar_response_device_health_t healthinfo;
        if (SL_IS_OK(drv->getHealth(healthinfo)) && healthinfo.status != SL_LIDAR_STATUS_ERROR) {
            return true;
        }
        fprintf(stderr, "LIDAR health check failed.\n");
        return false;
    }
};


// This is the module initialization function
PYBIND11_MODULE(rplidar, m) {
    m.doc() = "SLAMTEC LIDAR Python bindings"; // Optional docstring

    // Expose the LidarWrapper class to Python
    py::class_<LidarWrapper>(m, "LidarWrapper")
        .def(py::init<const std::string&, sl_u32>(), py::arg("port") = "/dev/ttyUSB0", py::arg("baudrate") = 1000000)
        .def("initialize", &LidarWrapper::initialize, "Initialize the Lidar")
        .def("get_scan_data", &LidarWrapper::getScanData, "Get scan data from the Lidar")
        .def("start_scan", &LidarWrapper::startScan, "Start scanning with the Lidar")
        .def("stop_scan", &LidarWrapper::stopScan, "Stop scanning with the Lidar");
}