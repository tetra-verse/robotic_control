#ifndef DEV_ROBOTINTERFACE_H_
#define DEV_ROBOTINTERFACE_H_

class RobotInterface
{
public:
    ~RobotInterface() = default;
    
    virtual bool connect() = 0;
    virtual bool disconnect() = 0;

    virtual int moveForward(float delta) = 0;

    virtual int moveJoint(float delta, int index) = 0;
};


#endif // DEV_ROBOTINTERFACE_H_
