#ifndef DEV_ROBOTXCOREDEV_H_
#define DEV_ROBOTXCOREDEV_H_

#include "rokae/robot.h"
#include "dev/robotinterface.h"

class RobotXCoreDev : public RobotInterface 
{
public:
    RobotXCoreDev(const std::string &remote_ip, const std::string& local_ip = "");
    ~RobotXCoreDev() override;

    bool connect() override;
    bool disconnect() override;

    int moveForward(float delta) override;
    int moveBackward(float delta) override;
    int moveLeftward(float delta) override;
    int moveRightward(float delta) override;
    int moveUpward(float delta) override;
    int moveDownward(float delta) override;

    int moveUp(float delta) override;
    int moveDown(float delta) override;
    int moveLeft(float delta) override;
    int moveRight(float delta) override;
    int moveRollClockwise(float delta) override;
    int moveRollCounterClockwise(float delta) override;

    int moveJoint(float delta, int index) override;
    int moveJoint(float *delta, int size) override;

    void setReadCallback(ReadCallback callback) override;

private:
    void setRobotMode();
    void setRobotReceiveState();
    int moveLine(float delta, int index);
    int moveDegree(float delta, int index);
    
    bool readRobotData(MotionData &data);

private:
    rokae::xMateRobot *robot_dev_ = nullptr; // Robot device pointer
    ReadCallback read_callback_ = nullptr; // Callback function for reading data

    std::string remote_ip_;
    std::string local_ip_;
    double speed_ = 0.5; // Default speed

};

#endif // DEV_ROBOTXCOREDEV_H_