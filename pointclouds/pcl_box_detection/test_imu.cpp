#include <chrono>
#include <cstdio>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <depthai/depthai.hpp>

int main() {
    using namespace std;
    using namespace std::chrono;

    // Create pipeline
    dai::Pipeline pipeline;

    // Define sources and outputs
    auto imu = pipeline.create<dai::node::IMU>();
    auto xlinkOut = pipeline.create<dai::node::XLinkOut>();

    xlinkOut->setStreamName("imu");

    imu->enableIMUSensor(dai::IMUSensor::LINEAR_ACCELERATION, 400);
    // enable GYROSCOPE_RAW at 400 hz rate
    // imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_RAW, 400);
    imu->enableIMUSensor(dai::IMUSensor::ROTATION_VECTOR, 400);
    // it's recommended to set both setBatchReportThreshold and setMaxBatchReports to 20 when integrating in a pipeline with a lot of input/output connections
    // above this threshold packets will be sent in batch of X, if the host is not blocked and USB bandwidth is available
    imu->setBatchReportThreshold(1);
    // maximum number of IMU packets in a batch, if it's reached device will block sending until host can receive it
    // if lower or equal to batchReportThreshold then the sending is always blocking on device
    // useful to reduce device's CPU load  and number of lost packets, if CPU load is high on device side due to multiple nodes
    imu->setMaxBatchReports(10);

    // Link plugins IMU -> XLINK
    imu->out.link(xlinkOut->input);

    // Pipeline is defined, now we can connect to the device
    dai::Device d(pipeline);

    bool firstTs = false;

    auto imuQueue = d.getOutputQueue("imu", 50, false);
    auto baseTs = std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration>();
    auto lastTs{baseTs};
    Eigen::Vector3f position{0.0f, 0.0f, 0.0f};
    Eigen::Vector3f velocity{0.0f, 0.0f, 0.0f};

    while(true) {
        auto imuData = imuQueue->get<dai::IMUData>();

        auto imuPackets = imuData->packets;
        for(auto& imuPacket : imuPackets) {
            // auto& acceleroValues = imuPacket.acceleroMeter;
            // auto& gyroValues = imuPacket.gyroscope;
            const Eigen::Quaternionf rotation{imuPacket.rotationVector.real, imuPacket.rotationVector.i, imuPacket.rotationVector.j, imuPacket.rotationVector.k};
            const Eigen::Vector3f gravity{0.0f, 0.0f, -9.81f};
            const Eigen::Vector3f rawAcceleration{imuPacket.acceleroMeter.x, imuPacket.acceleroMeter.y, imuPacket.acceleroMeter.z};
            const Eigen::Vector3f rotatedGravity{rotation * gravity};
            const Eigen::Vector3f fixedAcceleration{rotation * rawAcceleration};

            auto acceleroTs1 = imuPacket.acceleroMeter.getTimestampDevice();
            // auto gyroTs1 = gyroValues.getTimestampDevice();
            if(!firstTs) {
                baseTs = acceleroTs1;
                // baseTs = std::min(acceleroTs1, gyroTs1);
                firstTs = true;
            }

            auto acceleroTs = acceleroTs1 - baseTs;
            const float dt{duration_cast<microseconds>(acceleroTs1 - lastTs).count() / 1'000'000.0f};
            printf("%f\n", dt);
            velocity += dt * fixedAcceleration;
            position += dt * velocity;
            lastTs = acceleroTs1;
            // auto gyroTs = gyroTs1 - baseTs;

            printf("Accelerometer timestamp: %ld ms\n", static_cast<long>(duration_cast<milliseconds>(acceleroTs).count()));
            printf("Accelerometer [m/s^2]: x: %.3f y: %.3f z: %.3f \n", rawAcceleration.x(), rawAcceleration.y(), rawAcceleration.z());
            printf("Fixed accelerometer [m/s^2]: x: %.3f y: %.3f z: %.3f \n", fixedAcceleration.x(), fixedAcceleration.y(), fixedAcceleration.z());
            printf("Gravity [m/s^2]: x: %.3f y: %.3f z: %.3f \n", rotatedGravity.x(), rotatedGravity.y(), rotatedGravity.z());
            printf("Velocity [m]: x: %.3f y: %.3f z: %.3f \n", velocity.x(), velocity.y(), velocity.z());
            printf("Position [m]: x: %.3f y: %.3f z: %.3f \n", position.x(), position.y(), position.z());
            // printf("Gyroscope timestamp: %ld ms\n", static_cast<long>(duration_cast<milliseconds>(gyroTs).count()));
            // printf("Gyroscope [rad/s]: x: %.3f y: %.3f z: %.3f \n", gyroValues.x, gyroValues.y, gyroValues.z);
        }

        // int key = cv::waitKey(1);
        // if(key == 'q') {
        //     return 0;
        // }
    }

    return 0;
}
