#include "robotapp.h"


#include <thread>
#include <chrono>
#include <iostream>

#include "log/logging.h"
#include "dev/robotstudio.h"

void RobotApp::init()
{
    std::string remote_ip = "192.168.0.160";
    // RobotStudio dev(remote_ip);
    // dev.connect();

    RobotStudio krnx_dev(0, "127.0.0.1", 0);
    krnx_dev.connect();

    // dev.moveForward(10);
    // dev.moveBackward(10);
    // dev.moveLeftward(10);
    // dev.moveRightward(10);
    // dev.moveUpward(10);
    // dev.moveDownward(10);

    // dev.moveUp(10);
    // dev.moveDown(10);
    // dev.moveLeft(10);
    // dev.moveRight(10);
    // dev.moveRollClockwise(10);
    // dev.moveRollCounterClockwise(10);

    krnx_dev.setReadCallback([](const MotionData &data) {
        // Process the motion data received from the robot
        LOG_INFO("Received motion data: Angles: [0]:{} [1]:{} [2]:{} [3]:{} [4]:{} [5]:{}",
                 data.angles[0], data.angles[1], data.angles[2],
                 data.angles[3], data.angles[4], data.angles[5]);
    });

    int j = 0;
    float comp_max[6] = {0.01, 0.02, 0.03, 0.04, 0.03, 0.02};
    krnx_dev.moveSpeed(comp_max, sizeof(comp_max) / sizeof(comp_max[0]));
    for (int i = 0; i < 20; ++i) {
        if (i % 100 == 0) {
            j++;
        }
        // krnx_dev.moveJoint(0.001, 1); // Move each joint by 0.01 degrees
        // krnx_dev.moveJoint(comp_max, sizeof(comp_max));
        std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Sleep for 100 milliseconds
    }

    // dev.disconnect();
}

void RobotApp::start()
{
}

