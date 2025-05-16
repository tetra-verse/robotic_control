#ifndef DEV_ROBOTINTERFACE_H_
#define DEV_ROBOTINTERFACE_H_

class RobotInterface
{
public:
    ~RobotInterface() = default;
    
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
};


#endif // DEV_ROBOTINTERFACE_H_
