#ifndef DEV_ROBOTCONTROLLER_H_
#define DEV_ROBOTCONTROLLER_H_

#include <map>
#include <string>
#include <atomic>
#include <shared_mutex>

#include "dev/robotinterface.h"

enum class RobotType
{
    ROBOT_XCORE,
    ROBOT_KRNX
};

class RobotController
{
public:
    static RobotController& instance();

    int connect(RobotType type, const std::string &host_name, const std::string &local_ip = "");
    bool disconnect(int fd);

    int moveJoint(int fd, float delta, int index);

    int moveForward(int fd, float delta);

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
