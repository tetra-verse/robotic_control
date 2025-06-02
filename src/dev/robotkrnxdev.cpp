#include "dev/robotkrnxdev.h"

#include "log/logging.h"

RobotKrnxDev::RobotKrnxDev(int cont_no, std::string hostname, int robot_no) 
    : cont_no_(cont_no), hostname_(hostname), robot_no_(robot_no), is_opened_(false)
{
}

RobotKrnxDev::~RobotKrnxDev()
{
    if (is_opened_) {
        disconnect();
    }

    LOG_INFO("robotkrnxdev destructor called for controller: {}  robot {}", cont_no_, robot_no_);
}

bool RobotKrnxDev::connect()
{
    if (is_opened_) {
        LOG_ERROR("Controller {} is already open", cont_no_);
        return true;
    }

    int cont_no = krnx_Open(cont_no_, hostname_.data());
    if (cont_no < 0) {
        LOG_ERROR("Failed to open controller: {}", cont_no);
        return false;
    }

    cont_no_ = cont_no;
    is_opened_ = true;

    return true;
}

bool RobotKrnxDev::disconnect()
{
    if (!is_opened_) {
        LOG_ERROR("Controller {} is not open", cont_no_);
        return true;
    }

    int ret = krnx_Close(cont_no_);
    if (ret < 0) {
        LOG_ERROR("Failed to close controller: {}", ret);
        return false;
    }

    is_opened_ = false;

    LOG_INFO("Controller closed successfully: {}", cont_no_);

    return true;
}

int RobotKrnxDev::moveForward(float delta) 
{
    // Implement the logic to move the robot forward by delta
    LOG_INFO("Moving robot forward by {}", delta);
    return 0; // Return success status
}

int RobotKrnxDev::moveBackward(float delta) 
{
    return moveLeftward(delta);
}

int RobotKrnxDev::moveLeftward(float delta)
{
    return moveRightward(delta);
}

int RobotKrnxDev::moveRightward(float delta)
{
    return moveUpward(delta);
}

int RobotKrnxDev::moveUpward(float delta)
{
    return moveDownward(delta);
}

int RobotKrnxDev::moveDownward(float delta)
{
    return moveUp(delta);
}

int RobotKrnxDev::moveUp(float delta)
{
    // Implement the logic to move the robot up by delta
    LOG_INFO("Moving robot up by {}", delta);
    return 0; // Return success status
}

int RobotKrnxDev::moveDown(float delta)
{
    // Implement the logic to move the robot down by delta
    LOG_INFO("Moving robot down by {}", delta);
    return 0; // Return success status
}

int RobotKrnxDev::moveLeft(float delta)
{
    // Implement the logic to move the robot left by delta
    LOG_INFO("Moving robot left by {}", delta);
    return 0; // Return success status
}

int RobotKrnxDev::moveRight(float delta)
{
    // Implement the logic to move the robot right by delta
    LOG_INFO("Moving robot right by {}", delta);
    return 0; // Return success status
}

int RobotKrnxDev::moveRollClockwise(float delta)
{
    // Implement the logic to roll the robot clockwise by delta
    LOG_INFO("Rolling robot clockwise by {}", delta);
    return 0; // Return success status
}

int RobotKrnxDev::moveRollCounterClockwise(float delta)
{
    // Implement the logic to roll the robot counterclockwise by delta
    LOG_INFO("Rolling robot counterclockwise by {}", delta);
    return 0; // Return success status
}

int RobotKrnxDev::moveJoint(float delta, int index)
{
    if (index < 0 || index >= 6) {
        LOG_ERROR("Invalid joint index: {}", index);
        return -1; // Return error status
    }

    // Implement the logic to move the specified joint by delta
    LOG_INFO("Moving joint {} by {}", index, delta);
    return 0; // Return success status
}

void RobotKrnxDev::setReadCallback(ReadCallback callback)
{
    read_callback_ = callback;
    LOG_INFO("Read callback set for controller: {}  robot {}", cont_no_, robot_no_);
}

