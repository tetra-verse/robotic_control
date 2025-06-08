#ifndef DEV_ROBOTCONTROLLER_H_
#define DEV_ROBOTCONTROLLER_H_

#include <map>
#include <string>
#include <atomic>
#include <shared_mutex>

#include "dev/robotinterface.h"



class RobotController
{
public:
    static RobotController& instance();

    int connect(const std::string &host_name, const std::string &local_ip = "");
    int connect(int cont_no, std::string hostname, int robot_no = 0);
    bool disconnect(int fd);

    int moveJoint(int fd, float delta, int index);
    int moveJoint(int fd, float *delta, int size);

    int moveForward(int fd, float delta);
    int moveBackward(int fd, float delta);
    int moveLeftward(int fd, float delta);
    int moveRightward(int fd, float delta);
    int moveUpward(int fd, float delta);
    int moveDownward(int fd, float delta);

    int moveUp(int fd, float delta);
    int moveDown(int fd, float delta);
    int moveLeft(int fd, float delta);
    int moveRight(int fd, float delta);
    int moveRollClockwise(int fd, float delta);
    int moveRollCounterClockwise(int fd, float delta);

    void setReadCallback(int fd, ReadCallback callback);

private:
    RobotController() = default;
    ~RobotController();

    RobotInterface *findRobotDev(int fd);

private:
    std::atomic<int> fds_{0}; 
    mutable std::shared_mutex devs_mutex_;
    std::map<int, RobotInterface *> robot_devs_;

};

#endif // DEV_ROBOTCONTROLLER_H_
