#ifndef DEV_ROBOTKRNXDEV_H_
#define DEV_ROBOTKRNXDEV_H_

#include <string>
#include "dev/robotinterface.h"
#include "krnx.h"

enum ControllerMode
{
    AsExecutor = 0,
    RtcController
};

enum KrnxInternalSignals
{
  SignalControllerMode = 2010,
  SignalAsProgExecuting = 2011,
  SignalAsProgStarted = 2012,
  // signals are reserved for keyframe
  SignalKeyFrameStart = 2100,
  SignalKeyFrameEnd = 2612,
  SignalMax = 2001 + 960,
};


class RobotKrnxDev : public RobotInterface 
{
public:
    RobotKrnxDev(int cont_no, std::string hostname, int robot_no = 0);
    ~RobotKrnxDev() override;

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
    bool motorPower(bool on = true);
    bool switchMode(ControllerMode mode = RtcController);
    bool setSignal(int sig_no, bool on = true);
    bool activateRtcMode();
    bool deactivateRtcMode();
    bool runProgram();

    bool setRtcCompData(const float *comp6, int *status6);
    bool getRtcCompData(float *comp6);

    bool loadProgram(const std::string &prog_name, const std::string &program, const std::string &suffix);
    bool executeOnce(std::string_view program_name);
    bool krnxExecute(std::string_view program_name, int exec_num, int step_num);
    bool krnxErrorReset();

    bool setRtcInfo(TKrnxRtcInfo rtc_info);
    bool setMonitorSpeed(float speed = 10.0f);
    bool krnxOldCompClear();

    bool krnxHold();
    bool krnxKill();
    bool getRtcSwitch(int *rct_sw);

    bool getCurMotionDataEx(TKrnxCurMotionDataEx &motion_data);

    void readRobotData();
    void readDevCallback();
    bool parseRtcMotionDataEx(const TKrnxCurMotionDataEx &motion_data_ex, MotionData &motion_data);

private:
    int cont_no_ = -1; // Controller number
    int robot_no_ = -1; // Robot number
    std::string hostname_ = ""; // Hostname
    bool is_opened_ = false; // Opened status
    ReadCallback read_callback_ = nullptr; // Read callback function
    unsigned short seq_no_ = 0; // Sequence number for RTC comp data
};

#endif // DEV_ROBOTKRNXDEV_H_
