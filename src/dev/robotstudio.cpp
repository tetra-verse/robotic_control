#include "dev/robotstudio.h"

#include "log/logging.h"
#include "dev/robotcontroller.h"

RobotStudio::RobotStudio(std::string hostname, std::string local_ip) 
    : hostname_(hostname), local_ip_(local_ip), dev_type_(RobotType::ROBOT_XCORE)
{
    // Constructor implementation
}

RobotStudio::RobotStudio(int cont_no, std::string hostname, int robot_no)
    : cont_no_(cont_no), hostname_(hostname), robot_no_(robot_no), dev_type_(RobotType::ROBOT_KRNX)
{
    // Constructor implementation
}

RobotStudio::~RobotStudio() 
{
    disconnect();
}

bool RobotStudio::connect()
{
    int fd = -1;
    RobotController &controller = RobotController::instance();

    switch (dev_type_)
    {
        case RobotType::ROBOT_XCORE:
            fd = controller.connect(hostname_, local_ip_);
            break;
        case RobotType::ROBOT_KRNX:
            fd = controller.connect(cont_no_, hostname_, robot_no_);
            break;
        default:
            LOG_ERROR("Invalid robot type");
            return false;
    }

    if (fd < 0)
    {
        LOG_ERROR("Failed to connect to robot studio");
        return false;
    }

    fd_ = fd;
    LOG_INFO("Connected to robot studio with fd: {}", fd_);
    return true;
}

bool RobotStudio::disconnect()
{
    if (!isConnected())
    {
        LOG_ERROR("Already disconnected from robot studio");
        return false;
    }

    RobotController &controller = RobotController::instance();
    if (!controller.disconnect(fd_))
    {
        LOG_ERROR("Failed to disconnect from robot studio with fd: {}", fd_);
        return false;
    }
    
    fd_ = -1;
    LOG_INFO("Disconnected from robot studio");
    return true;
}

int RobotStudio::moveForward(float delta)
{
    if (isConnected())
    {
        RobotController &controller = RobotController::instance();
        int ret = controller.moveForward(fd_, delta);
        LOG_INFO("Moved forward by {} mm ret {}", delta, ret);
        return ret;
    }


    LOG_ERROR("Not connected to robot studio");
    return -1;
}

int RobotStudio::moveBackward(float delta)
{
    if (isConnected())
    {
        RobotController &controller = RobotController::instance();
        int ret = controller.moveBackward(fd_, delta);
        LOG_INFO("Moved backward by {} mm ret {}", delta, ret);
        return ret;
    }

    LOG_ERROR("Not connected to robot studio");
    return -1;
}

int RobotStudio::moveLeftward(float delta)
{
    if (isConnected())
    {
        RobotController &controller = RobotController::instance();
        int ret = controller.moveLeftward(fd_, delta);
        LOG_INFO("Moved leftward by {} mm ret {}", delta, ret);
        return ret;
    }

    LOG_ERROR("Not connected to robot studio");
    return -1;
}

int RobotStudio::moveRightward(float delta)
{
    if (isConnected())
    {
        RobotController &controller = RobotController::instance();
        int ret = controller.moveRightward(fd_, delta);
        LOG_INFO("Moved rightward by {} mm ret {}", delta, ret);
        return ret;
    }

    LOG_ERROR("Not connected to robot studio");
    return -1;
}

int RobotStudio::moveUpward(float delta)
{
    if (isConnected())
    {
        RobotController &controller = RobotController::instance();
        int ret = controller.moveUpward(fd_, delta);
        LOG_INFO("Moved upward by {} mm ret {}", delta, ret);
        return ret;
    }

    LOG_ERROR("Not connected to robot studio");
    return -1;
}

int RobotStudio::moveDownward(float delta)
{
    if (isConnected())
    {
        RobotController &controller = RobotController::instance();
        int ret = controller.moveDownward(fd_, delta);
        LOG_INFO("Moved downward by {} mm ret {}", delta, ret);
        return ret;
    }

    LOG_ERROR("Not connected to robot studio");
    return -1;
}

int RobotStudio::moveJoint(float delta, int index)
{
    if (isConnected()) {
        RobotController &controller = RobotController::instance();
        int ret = controller.moveJoint(fd_, delta, index);
        LOG_INFO("Moved joint {} by {} degrees ret {}", index, delta, ret);
        return ret;
    }

    LOG_ERROR("Not connected to robot studio");
    return -1;
}

int RobotStudio::moveJoint(float *delta, int size)
{
    if (isConnected()) {
        RobotController &controller = RobotController::instance();
        int ret = controller.moveJoint(fd_, delta, size);
        LOG_INFO("Moved joint {} degrees ret {}", size, ret);
        return ret;
    }

    LOG_ERROR("Not connected to robot studio");
    return -1;
}

int RobotStudio::moveUp(float delta)
{
    if (isConnected())
    {
        RobotController &controller = RobotController::instance();
        int ret = controller.moveUp(fd_, delta);
        LOG_INFO("Moved up by {} degrees ret {}", delta, ret);
        return ret;
    }

    LOG_ERROR("Not connected to robot studio");
    return -1;
}

int RobotStudio::moveDown(float delta)
{
    if (isConnected())
    {
        RobotController &controller = RobotController::instance();
        int ret = controller.moveDown(fd_, delta);
        LOG_INFO("Moved down by {} degrees ret {}", delta, ret);
        return ret;
    }

    LOG_ERROR("Not connected to robot studio");
    return -1;
}

int RobotStudio::moveLeft(float delta)
{
    if (isConnected())
    {
        RobotController &controller = RobotController::instance();
        int ret = controller.moveLeft(fd_, delta);
        LOG_INFO("Moved left by {} degrees ret {}", delta, ret);
        return ret;
    }

    LOG_ERROR("Not connected to robot studio");
    return -1;
}

int RobotStudio::moveRight(float delta)
{
    if (isConnected())
    {
        RobotController &controller = RobotController::instance();
        int ret = controller.moveRight(fd_, delta);
        LOG_INFO("Moved right by {} degrees ret {}", delta, ret);
        return ret;
    }

    LOG_ERROR("Not connected to robot studio");
    return -1;
}

int RobotStudio::moveRollClockwise(float delta)
{
    if (isConnected())
    {
        RobotController &controller = RobotController::instance();
        int ret = controller.moveRollClockwise(fd_, delta);
        LOG_INFO("Moved roll clockwise by {} degrees ret {}", delta, ret);
        return ret;
    }

    LOG_ERROR("Not connected to robot studio");
    return -1;
}

int RobotStudio::moveRollCounterClockwise(float delta)
{
    if (isConnected())
    {
        RobotController &controller = RobotController::instance();
        int ret = controller.moveRollCounterClockwise(fd_, delta);
        LOG_INFO("Moved roll counter-clockwise by {} degrees ret {}", delta, ret);
        return ret;
    }

    LOG_ERROR("Not connected to robot studio");
    return -1;
}

void RobotStudio::setReadCallback(ReadCallback callback)
{
    if (isConnected())
    {
        RobotController &controller = RobotController::instance();
        controller.setReadCallback(fd_, callback);
        LOG_INFO("Set read callback for robot studio with fd: {}", fd_);
        return;
    }

    LOG_ERROR("Not connected to robot studio");
}
