#include "dev/robotcontroller.h"


#include "log/logging.h"
#include "dev/robotxcoredev.h"
#include "dev/robotkrnxdev.h"

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

int RobotController::connect(const std::string &host_name, const std::string &local_ip)
{
    RobotInterface *robot_dev = new RobotXCoreDev(host_name, local_ip);
    if (robot_dev == nullptr || !robot_dev->connect()) {
        LOG_ERROR("Failed to create robot device ");
        delete robot_dev;
        return -1;
    }

    std::unique_lock<std::shared_mutex> lock(devs_mutex_);
    int fd = fds_++;
    robot_devs_[fd] = robot_dev;

    return fd;
}

int RobotController::connect(int cont_no, std::string hostname, int robot_no)
{
    RobotInterface *robot_dev = new RobotKrnxDev(cont_no, hostname, robot_no);
    if (robot_dev == nullptr || !robot_dev->connect()) {
        LOG_ERROR("Failed to create robot device");
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
    auto robot_dev = findRobotDev(fd);
    if (robot_dev == nullptr) {
        LOG_ERROR("Failed to move forward: invalid file descriptor {}", fd);
        return -1;
    }

    int ret = robot_dev->moveForward(delta); // Assuming 0 is the index for forward movement
    if (ret != 0) {
        LOG_ERROR("Failed to move forward: {}", ret);
        return -1;
    }

    return ret;
}

int RobotController::moveBackward(int fd, float delta)
{
    std::shared_lock<std::shared_mutex> lock(devs_mutex_);
    auto robot_dev = findRobotDev(fd);
    if (robot_dev == nullptr) {
        LOG_ERROR("Failed to move backward: invalid file descriptor {}", fd);
        return -1;
    }

    int ret = robot_dev->moveBackward(delta); // Assuming 0 is the index for backward movement
    if (ret != 0) {
        LOG_ERROR("Failed to move backward: {}", ret);
        return -1;
    }

    return ret;
}

int RobotController::moveLeftward(int fd, float delta)
{
    std::shared_lock<std::shared_mutex> lock(devs_mutex_);
    auto robot_dev = findRobotDev(fd);
    if (robot_dev == nullptr) {
        LOG_ERROR("Failed to move leftward: invalid file descriptor {}", fd);
        return -1;
    }

    int ret = robot_dev->moveLeftward(delta); // Assuming 1 is the index for leftward movement
    if (ret != 0) {
        LOG_ERROR("Failed to move leftward: {}", ret);
        return -1;
    }

    return ret;
}

int RobotController::moveRightward(int fd, float delta)
{
    std::shared_lock<std::shared_mutex> lock(devs_mutex_);
    auto robot_dev = findRobotDev(fd);
    if (robot_dev == nullptr) {
        LOG_ERROR("Failed to move rightward: invalid file descriptor {}", fd);
        return -1;
    }

    int ret = robot_dev->moveRightward(delta); // Assuming 1 is the index for rightward movement
    if (ret != 0) {
        LOG_ERROR("Failed to move rightward: {}", ret);
        return -1;
    }

    return ret;
}

int RobotController::moveUpward(int fd, float delta)
{
    std::shared_lock<std::shared_mutex> lock(devs_mutex_);
    auto robot_dev = findRobotDev(fd);
    if (robot_dev == nullptr) {
        LOG_ERROR("Failed to move upward: invalid file descriptor {}", fd);
        return -1;
    }

    int ret = robot_dev->moveUpward(delta); // Assuming 2 is the index for upward movement
    if (ret != 0) {
        LOG_ERROR("Failed to move upward: {}", ret);
        return -1;
    }

    return ret;
}

int RobotController::moveDownward(int fd, float delta)
{
    std::shared_lock<std::shared_mutex> lock(devs_mutex_);
    auto robot_dev = findRobotDev(fd);
    if (robot_dev == nullptr) {
        LOG_ERROR("Failed to move downward: invalid file descriptor {}", fd);
        return -1;
    }

    int ret = robot_dev->moveDownward(delta); // Assuming 2 is the index for downward movement
    if (ret != 0) {
        LOG_ERROR("Failed to move downward: {}", ret);
        return -1;
    }

    return ret;
}

int RobotController::moveJoint(int fd, float delta, int index)
{
    std::shared_lock<std::shared_mutex> lock(devs_mutex_);
    auto robot_dev = findRobotDev(fd);
    if (robot_dev == nullptr) {
        LOG_ERROR("Failed to move joint: invalid file descriptor {}", fd);
        return -1;
    }

    int ret = robot_dev->moveJoint(delta, index);
    if (ret != 0) {
        LOG_ERROR("Failed to move joint {}: {}", index, ret);
        return -1;
    }

    return ret;
}

int RobotController::moveJoint(int fd, float *delta, int size)
{
    std::shared_lock<std::shared_mutex> lock(devs_mutex_);
    auto robot_dev = findRobotDev(fd);
    if (robot_dev == nullptr) {
        LOG_ERROR("Failed to move joint: invalid file descriptor {}", fd);
        return -1;
    }

    int ret = robot_dev->moveJoint(delta, size);
    if (ret != 0) {
        LOG_ERROR("Failed to move joint {}", ret);
        return -1;
    }

    return ret;
}

int RobotController::moveSpeed(int fd, float *delta, int size)
{
    std::shared_lock<std::shared_mutex> lock(devs_mutex_);
    auto robot_dev = findRobotDev(fd);
    if (robot_dev == nullptr) {
        LOG_ERROR("Failed to move speed: invalid file descriptor {}", fd);
        return -1;
    }

    int ret = robot_dev->moveSpeed(delta, size);
    if (ret != 0) {
        LOG_ERROR("Failed to move speed: {}", ret);
        return -1;
    }

    return ret;
}

int RobotController::moveUp(int fd, float delta)
{
    std::shared_lock<std::shared_mutex> lock(devs_mutex_);
    auto robot_dev = findRobotDev(fd);
    if (robot_dev == nullptr) {
        LOG_ERROR("Failed to move up: invalid file descriptor {}", fd);
        return -1;
    }

    int ret = robot_dev->moveUp(delta); // Assuming 3 is the index for upward movement
    if (ret != 0) {
        LOG_ERROR("Failed to move up: {}", ret);
        return -1;
    }

    return ret;
}

int RobotController::moveDown(int fd, float delta)
{
    std::shared_lock<std::shared_mutex> lock(devs_mutex_);
    auto robot_dev = findRobotDev(fd);
    if (robot_dev == nullptr) {
        LOG_ERROR("Failed to move down: invalid file descriptor {}", fd);
        return -1;
    }

    int ret = robot_dev->moveDown(delta); // Assuming 3 is the index for downward movement
    if (ret != 0) {
        LOG_ERROR("Failed to move down: {}", ret);
        return -1;
    }

    return ret;
}

int RobotController::moveLeft(int fd, float delta)
{
    std::shared_lock<std::shared_mutex> lock(devs_mutex_);
    auto robot_dev = findRobotDev(fd);
    if (robot_dev == nullptr) {
        LOG_ERROR("Failed to move left: invalid file descriptor {}", fd);
        return -1;
    }

    int ret = robot_dev->moveLeft(delta); // Assuming 4 is the index for leftward movement
    if (ret != 0) {
        LOG_ERROR("Failed to move left: {}", ret);
        return -1;
    }

    return ret;
}

int RobotController::moveRight(int fd, float delta)
{
    std::shared_lock<std::shared_mutex> lock(devs_mutex_);
    auto robot_dev = findRobotDev(fd);
    if (robot_dev == nullptr) {
        LOG_ERROR("Failed to move right: invalid file descriptor {}", fd);
        return -1;
    }

    int ret = robot_dev->moveRight(delta); // Assuming 4 is the index for rightward movement
    if (ret != 0) {
        LOG_ERROR("Failed to move right: {}", ret);
        return -1;
    }

    return ret;
}

int RobotController::moveRollClockwise(int fd, float delta)
{
    std::shared_lock<std::shared_mutex> lock(devs_mutex_);
    auto robot_dev = findRobotDev(fd);
    if (robot_dev == nullptr) {
        LOG_ERROR("Failed to move roll clockwise: invalid file descriptor {}", fd);
        return -1;
    }

    int ret = robot_dev->moveRollClockwise(delta); // Assuming 5 is the index for clockwise roll
    if (ret != 0) {
        LOG_ERROR("Failed to move roll clockwise: {}", ret);
        return -1;
    }

    return ret;
}

int RobotController::moveRollCounterClockwise(int fd, float delta)
{
    std::shared_lock<std::shared_mutex> lock(devs_mutex_);
    auto robot_dev = findRobotDev(fd);
    if (robot_dev == nullptr) {
        LOG_ERROR("Failed to move roll counterclockwise: invalid file descriptor {}", fd);
        return -1;
    }

    int ret = robot_dev->moveRollCounterClockwise(delta); // Assuming 5 is the index for counterclockwise roll
    if (ret != 0) {
        LOG_ERROR("Failed to move roll counterclockwise: {}", ret);
        return -1;
    }

    return ret;
}

void RobotController::setReadCallback(int fd, ReadCallback callback)
{
    std::shared_lock<std::shared_mutex> lock(devs_mutex_);
    auto robot_dev = findRobotDev(fd);
    if (robot_dev == nullptr) {
        LOG_ERROR("Failed to set read callback: invalid file descriptor {}", fd);
        return;
    }

    robot_dev->setReadCallback(callback);
}

