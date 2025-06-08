#ifndef DEV_ROBOTINTERFACE_H_
#define DEV_ROBOTINTERFACE_H_

#include <array>
#include <functional>

struct MotionData
{
    std::array<float, 6> angles = {0};
    std::array<float, 6> xyzoat = {0};
    std::array<float, 6> xyzptr = {0};
};

using ReadCallback = std::function<void(const MotionData &)>;

class RobotInterface
{
public:
    virtual ~RobotInterface() {};
    
    virtual bool connect() = 0;
    virtual bool disconnect() = 0;

    virtual int moveForward(float delta) = 0;
    virtual int moveBackward(float delta) = 0;
    virtual int moveLeftward(float delta) = 0;
    virtual int moveRightward(float delta) = 0;
    virtual int moveUpward(float delta) = 0;
    virtual int moveDownward(float delta) = 0;

    virtual int moveUp(float delta) = 0;
    virtual int moveDown(float delta) = 0;
    virtual int moveLeft(float delta) = 0;
    virtual int moveRight(float delta) = 0;
    virtual int moveRollClockwise(float delta) = 0;
    virtual int moveRollCounterClockwise(float delta) = 0;

    virtual int moveJoint(float delta, int index) = 0;
    virtual int moveJoint(float *delta, int size) = 0;

    virtual void setReadCallback(ReadCallback callback) = 0;
};


#endif // DEV_ROBOTINTERFACE_H_
