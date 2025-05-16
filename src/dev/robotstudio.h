#ifndef DEV_ROBOTSTUDIO_H_
#define DEV_ROBOTSTUDIO_H_

#include <string>


class RobotStudio
{
public:
    RobotStudio(std::string hostname, std::string local_ip = "");
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

private:
    std::string hostname_ = ""; // Hostname
    std::string local_ip_ = ""; // Local IP address
    int fd_ = -1; // File descriptor for the connection
};



#endif // DEV_ROBOTSTUDIO_H_