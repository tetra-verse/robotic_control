#include "dev/robotstudio.h"

#include "log/logging.h"
#include "dev/robotcontroller.h"

RobotStudio::RobotStudio(std::string hostname, std::string local_ip) 
    : hostname_(hostname), local_ip_(local_ip)
{
    // Constructor implementation
}

RobotStudio::~RobotStudio() 
{
    disconnect();
}

bool RobotStudio::connect()
{
    RobotController &controller = RobotController::instance();
    int fd = controller.connect(RobotType::ROBOT_XCORE, hostname_, local_ip_);
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
    if (fd_ < 0)
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
    if (fd_ >= 0)
    {
        RobotController &controller = RobotController::instance();
        int ret = controller.moveForward(fd_, delta);
        LOG_INFO("Moved forward by {} mm ret {}", delta, ret);
        return ret;
    }


    LOG_ERROR("Not connected to robot studio");
    return -1;
}


int RobotStudio::moveJoint(float delta, int index)
{
    if (fd_ >= 0)
    {
        RobotController &controller = RobotController::instance();
        int ret = controller.moveJoint(fd_, delta, index);
        LOG_INFO("Moved joint {} by {} degrees ret {}", index, delta, ret);
        return ret;
    }

    LOG_ERROR("Not connected to robot studio");
    return -1;
}
