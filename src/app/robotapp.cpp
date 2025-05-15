#include "robotapp.h"

#include "dev/robotstudio.h"

void RobotApp::init()
{
    std::string remote_ip = "192.168.0.160";
    RobotStudio dev(remote_ip);
    dev.connect();

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

    for (int i = 0; i < 6; ++i) {
        dev.moveJoint(10, i);
    }

    dev.disconnect();
}

void RobotApp::start()
{
}

