#ifndef DEV_ROBOTKRNXDEV_H_
#define DEV_ROBOTKRNXDEV_H_

#include <string>
#include "dev/robotinterface.h"
#include "krnx.h"


class RobotKrnxDev : public RobotInterface 
{
public:
    RobotKrnxDev(int cont_no, std::string hostname, int robot_no = 0);
    ~RobotKrnxDev();

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

    void setReadCallback(ReadCallback callback) override;

private:
    int cont_no_ = -1; // Controller number
    int robot_no_ = -1; // Robot number
    std::string hostname_ = ""; // Hostname
    bool is_opened_ = false; // Opened status
    ReadCallback read_callback_ = nullptr; // Read callback function
};

#endif // DEV_ROBOTKRNXDEV_H_
