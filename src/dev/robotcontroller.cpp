#include "dev/robotcontroller.h"


#include "log/logging.h"
#include "dev/robotxcoredev.h"

RobotController& RobotController::instance()
{
    static RobotController instance;
    return instance;
}


RobotController::~RobotController()
{
    std::unique_lock<std::shared_mutex> lock(devs_mutex_);
    for (auto &pair : robot_devs_) {
        RobotInterface *robot_dev = pair.second;
        robot_dev->disconnect();
        delete robot_dev;
    }

    robot_devs_.clear();
}

int RobotController::connect(RobotType type, const std::string &host_name, const std::string &local_ip)
{
    RobotInterface *robot_dev = nullptr;

    switch (type) {
        case RobotType::ROBOT_XCORE:
            robot_dev = new RobotXCoreDev(host_name, local_ip);
            break;
        case RobotType::ROBOT_KRNX:
            // robot_dev = new RobotKrnxDev(host_name, local_ip);
            break;
        default:
            LOG_ERROR("Invalid robot type");
            return -1;
    }

    if (!robot_dev->connect()) {
        delete robot_dev;
        return -1;
    }

    std::unique_lock<std::shared_mutex> lock(devs_mutex_);
    int fd = fds_++;
    robot_devs_[fd] = robot_dev;

    return fd;
}

bool RobotController::disconnect(int fd)
{
    std::unique_lock<std::shared_mutex> lock(devs_mutex_);
    auto it = robot_devs_.find(fd);
    if (it != robot_devs_.end()) {
        RobotInterface *robot_dev = it->second;
        robot_dev->disconnect();
        delete robot_dev;
        robot_devs_.erase(it);
        return true;
    }

    LOG_ERROR("Failed to disconnect: invalid file descriptor {}", fd);
    return false;
}

RobotInterface *RobotController::findRobotDev(int fd)
{
    auto it = robot_devs_.find(fd);
    if (it != robot_devs_.end()){
        return it->second;
    }

    return nullptr;
}

int RobotController::moveForward(int fd, float delta)
{
    std::shared_lock<std::shared_mutex> lock(devs_mutex_);
    auto it = robot_devs_.find(fd);
    if (it == robot_devs_.end()) {
        LOG_ERROR("Failed to move forward: invalid file descriptor {}", fd);
        return -1;
    }

    RobotInterface *robot_dev = it->second;
    int ret = robot_dev->moveJoint(delta, 0); // Assuming 0 is the index for forward movement
    if (ret != 0) {
        LOG_ERROR("Failed to move forward: {}", ret);
        return -1;
    }

    return ret;
}

int RobotController::moveJoint(int fd, float delta, int index)
{
    std::shared_lock<std::shared_mutex> lock(devs_mutex_);
    auto it = robot_devs_.find(fd);
    if (it == robot_devs_.end()) {
        LOG_ERROR("Failed to move joint: invalid file descriptor {}", fd);
        return -1;
    }

    RobotInterface *robot_dev = it->second;
    int ret = robot_dev->moveJoint(delta, index);
    if (ret != 0) {
        LOG_ERROR("Failed to move joint {}: {}", index, ret);
        return -1;
    }

    return ret;
}

