#ifndef DEV_ROBOTXCOREDEV_H_
#define DEV_ROBOTXCOREDEV_H_

#include "dev/robotinterface.h"
#include "rokae/robot.h"


class RobotXCoreDev : public RobotInterface
{
public:
    RobotXCoreDev(const std::string &remote_ip, const std::string &local_ip = "");
    ~RobotXCoreDev();

    bool connect() override;
    bool disconnect() override;

    int moveForward(float delta) override;

    int moveJoint(float delta, int index) override;

private:
    void setRobotMode();
    int moveLine(float delta, int index);

private:
    rokae::xMateRobot *robot_dev_ = nullptr;  // Robot device pointer

    std::string remote_ip_;
    std::string local_ip_;
    double speed_ = 0.5;  // Default speed
};

#endif  // DEV_ROBOTXCOREDEV_H_