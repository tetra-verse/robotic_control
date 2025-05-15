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
    
    // space mm
    int moveForward(float delta);
    void moveBackward(float delta);
    void moveLeftward(float delta);
    void moveRightward(float delta);
    void moveUpward(float delta);
    void moveDownward(float delta);

    // degrees
    void moveUp(float delta);
    void moveDown(float delta);
    void moveLeft(float delta);
    void moveRight(float delta);
    void moveRollClockwise(float delta);
    void moveRollCounterClockwise(float delta);

    // degrees
    int moveJoint(float delta, int index);

private:
    std::string hostname_ = ""; // Hostname
    std::string local_ip_ = ""; // Local IP address
    int fd_ = -1; // File descriptor for the connection
};



#endif // DEV_ROBOTSTUDIO_H_