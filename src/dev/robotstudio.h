#ifndef DEV_ROBOTSTUDIO_H_
#define DEV_ROBOTSTUDIO_H_

#include <string>

#include "dev/robotinterface.h"

enum class RobotType
{
    ROBOT_XCORE,
    ROBOT_KRNX
};

class RobotStudio
{
public:
    RobotStudio(std::string hostname, std::string local_ip = "");
    RobotStudio(int cont_no, std::string hostname, int robot_no = 0);
    ~RobotStudio();

    bool connect();
    bool disconnect();
    bool isConnected() const { return fd_ >= 0; }
    
    // space mm
    int moveForward(float delta);
    int moveBackward(float delta);
    int moveLeftward(float delta);
    int moveRightward(float delta);
    int moveUpward(float delta);
    int moveDownward(float delta);

    // degrees
    int moveUp(float delta);
    int moveDown(float delta);
    int moveLeft(float delta);
    int moveRight(float delta);
    int moveRollClockwise(float delta);
    int moveRollCounterClockwise(float delta);

    // degrees
    int moveJoint(float delta, int index);
    int moveJoint(float *delta, int size);

    int moveSpeed(float *delta, int size);

    void setReadCallback(ReadCallback callback);

private:
    std::string hostname_ = ""; // Hostname
    std::string local_ip_ = ""; // Local IP address
    int fd_ = -1; // File descriptor for the connection

    // Controller and Robot numbers`
    int cont_no_ = -1; // Controller number
    int robot_no_ = -1; // Robot number
    RobotType dev_type_ = RobotType::ROBOT_XCORE; // Robot type
};



#endif // DEV_ROBOTSTUDIO_H_