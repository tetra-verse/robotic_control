#include "dev/robotkrnxdev.h"

#include "log/logging.h"

RobotKrnxDev::RobotKrnxDev(int cont_no, std::string hostname, int robot_no) 
    : cont_no_(cont_no), hostname_(hostname), robot_no_(robot_no), is_opened_(false)
{
}

RobotKrnxDev::~RobotKrnxDev()
{
    if (is_opened_) {
        disconnect();
    }

    LOG_INFO("robotkrnxdev destructor called for controller: {}  robot {}", cont_no_, robot_no_);
}

bool RobotKrnxDev::connect()
{
    if (is_opened_) {
        LOG_ERROR("Controller {} is already open", cont_no_);
        return true;
    }

    int cont_no = krnx_Open(cont_no_, hostname_.data());
    if (cont_no < 0) {
        LOG_ERROR("Failed to open controller: {}", cont_no);
        return false;
    }

    cont_no_ = cont_no;
    is_opened_ = true;

    if (!runProgram()) {
        disconnect();
        LOG_ERROR("Failed to run program on controller: {}", cont_no_);
        return false;
    }

    return true;
}

bool RobotKrnxDev::disconnect()
{
    if (!is_opened_) {
        LOG_ERROR("Controller {} is not open", cont_no_);
        return true;
    }

    int ret = krnx_Close(cont_no_);
    if (ret < 0) {
        LOG_ERROR("Failed to close controller: {}", ret);
        return false;
    }

    is_opened_ = false;

    LOG_INFO("Controller closed successfully: {}", cont_no_);

    return true;
}

int RobotKrnxDev::moveForward(float delta) 
{
    // Implement the logic to move the robot forward by delta
    LOG_INFO("Moving robot forward by {}", delta);
    return 0; // Return success status
}

int RobotKrnxDev::moveBackward(float delta) 
{
    return moveLeftward(delta);
}

int RobotKrnxDev::moveLeftward(float delta)
{
    return moveRightward(delta);
}

int RobotKrnxDev::moveRightward(float delta)
{
    return moveUpward(delta);
}

int RobotKrnxDev::moveUpward(float delta)
{
    return moveDownward(delta);
}

int RobotKrnxDev::moveDownward(float delta)
{
    return moveUp(delta);
}

int RobotKrnxDev::moveUp(float delta)
{
    // Implement the logic to move the robot up by delta
    LOG_INFO("Moving robot up by {}", delta);
    return 0; // Return success status
}

int RobotKrnxDev::moveDown(float delta)
{
    // Implement the logic to move the robot down by delta
    LOG_INFO("Moving robot down by {}", delta);
    return 0; // Return success status
}

int RobotKrnxDev::moveLeft(float delta)
{
    // Implement the logic to move the robot left by delta
    LOG_INFO("Moving robot left by {}", delta);
    return 0; // Return success status
}

int RobotKrnxDev::moveRight(float delta)
{
    // Implement the logic to move the robot right by delta
    LOG_INFO("Moving robot right by {}", delta);
    return 0; // Return success status
}

int RobotKrnxDev::moveRollClockwise(float delta)
{
    // Implement the logic to roll the robot clockwise by delta
    LOG_INFO("Rolling robot clockwise by {}", delta);
    return 0; // Return success status
}

int RobotKrnxDev::moveRollCounterClockwise(float delta)
{
    // Implement the logic to roll the robot counterclockwise by delta
    LOG_INFO("Rolling robot counterclockwise by {}", delta);
    return 0; // Return success status
}

int RobotKrnxDev::moveJoint(float delta, int index)
{
    if (index < 0 || index >= 6) {
        LOG_ERROR("Invalid joint index: {}", index);
        return -1; // Return error status
    }

    float comp_max[18] = {0};
    int status_max[18] = {0};
    if (getRtcCompData(comp_max) == false) {
        LOG_ERROR("Failed to get RTC comp data for controller: {}", cont_no_);
        return -1; // Return error status
    }

    comp_max[index] += delta; // Set the delta for the specified joint
    if (!setRtcCompData(comp_max, status_max)) {
        LOG_ERROR("Failed to set RTC comp data for controller: {}", cont_no_);
        return -1; // Return error status
    }

    return 0; // Return success status
}

int RobotKrnxDev::moveJoint(float *delta, int size)
{
    float comp_max[18] = {0};
    int status_max[18] = {0};

    if (getRtcCompData(comp_max) == false) {
        LOG_ERROR("Failed to get RTC comp data for controller: {}", cont_no_);
        return -1; // Return error status
    }

    for (int i = 0; i < sizeof(comp_max) && i < size; i++) {
        comp_max[i] += delta[i];
    }
    
    if (!setRtcCompData(comp_max, status_max)) {
        LOG_ERROR("Failed to set RTC comp data for controller: {}", cont_no_);
        return -1; // Return error status
    }

    return 0; // Return success status
}

int RobotKrnxDev::moveSpeed(float *delta, int size)
{
    int index = 0;
    for (; index < size && index < 6; ++index) {
        delta_angle_[index] = delta[index];
    }

    for (; index < 6; ++index) {
        delta_angle_[index] = 0.0; // Fill remaining angles with zero
    }

    return 0;
}

bool RobotKrnxDev::setRtcCompData(const float *comp6, int *status6)
{
    int ret = krnx_SetRtcCompData(cont_no_, robot_no_, comp6, status6, seq_no_);
    if (ret < 0) {
        LOG_ERROR("Failed to set RTC comp data: {} seq no {}", ret, seq_no_);
        return false;
    }

    LOG_INFO("RTC comp data set successfully for controller: {} seq no {}", cont_no_, seq_no_);
    seq_no_++;
    return true;
}

bool RobotKrnxDev::getRtcCompData(float *comp6)
{
    int ret = krnx_GetRtcCompData(cont_no_, robot_no_, comp6);
    if (ret < 0) {
        LOG_ERROR("Failed to get RTC comp data: {}", ret);
        return false;
    }

    LOG_INFO("RTC comp data retrieved successfully for controller: {}", cont_no_);
    return true;
}

void RobotKrnxDev::setReadCallback(ReadCallback callback)
{
    read_callback_ = callback;
    LOG_INFO("Read callback set for controller: {}  robot {}", cont_no_, robot_no_);
}

bool RobotKrnxDev::motorPower(bool on)
{
    int err_code = 0; // Error code pointer, can be used for error handling
    char buffer[256] = {0}; // Buffer to store the response
    const char *monitor_cmd = on ? "ZPOW ON" : "ZPOW OFF";
    int ret = krnx_ExecMon(cont_no_, monitor_cmd, buffer, sizeof(buffer) - 1, &err_code);
    if (ret < 0) {
        LOG_ERROR("Failed to set motor power: {} err coce {}", ret, err_code);
        return false;
    }

    LOG_INFO("Motor power set successfully for controller: {}", cont_no_);
    return true;
}

bool RobotKrnxDev::setSignal(int sig_no, bool on)
{
    int err_code = 0; // Error code pointer, can be used for error handling
    int status = on ? -1 : 0;
    int ret = krnx_SetSignal(cont_no_, sig_no, status, &err_code);
    if (ret < 0) {
        LOG_ERROR("Failed to set signal: {} err code {}", ret, err_code);
        return false;
    }

    LOG_INFO("Signal set successfully for controller: {}", cont_no_);
    return true;
}

bool RobotKrnxDev::switchMode(ControllerMode mode)
{
    if (!setSignal(SignalAsProgExecuting, mode == RtcController)) {
        LOG_ERROR("Failed to switch controller mode: rtc");
        return false;
    }

    LOG_INFO("Controller mode switched successfully for controller: {}", cont_no_);
    return true;
}

bool RobotKrnxDev::loadProgram(const std::string &prog_name, const std::string &program, const std::string &suffix)
{
    std::string full_program_name = prog_name + suffix;
    int ret = krnx_Load(cont_no_, full_program_name.data());
    if (ret < 0) {
        LOG_ERROR("Failed to load program: {}", ret);
        return false;
    }

    LOG_INFO("Program loaded successfully for controller: {} program {}", cont_no_, full_program_name);
    return true;
}

bool RobotKrnxDev::executeOnce(std::string_view program_name)
{
    return krnxExecute(program_name, 1, 0);
}

bool RobotKrnxDev::krnxExecute(std::string_view program_name, int exec_num, int step_num)
{
    int err_code = 0; // Error code pointer, can be used for error handling
    int ret = krnx_Execute(cont_no_, robot_no_, program_name.data(), exec_num, step_num, &err_code);
    if (ret < 0) {
        LOG_ERROR("Failed to execute program: {} err code {}", ret, err_code);
        return false;
    }

    LOG_INFO("Program executed successfully for controller: {}", cont_no_);
    return true;
}

bool RobotKrnxDev::activateRtcMode()
{
    TKrnxRtcInfo rtc_info = {8, 4, 1}; // Example RTC info
    if (!setRtcInfo(rtc_info)) {
        LOG_ERROR("Failed to set RTC info: {}", cont_no_);
        return false;
    }

    std::stringstream ss;
    ss << ".PROGRAM gka_rb_rtc()\n"
        << "  HERE #rtchome1\n"
        << "  ACCURACY 0 ALWAYS\n"
        << "  RTC_SW 1: ON\n"
        << "  1\n"
        << "  JMOVE #rtchome1\n"
        << "  GOTO 1\n"
        << "  RTC_SW 1: OFF\n"
        << ".END\n";

    std::string program = ss.str();
    if (!loadProgram("gka_rb_rtc", program, ".as")) {
        LOG_ERROR("Failed to load program for RTC mode: {}", cont_no_);
        return false;
    }

    // Reset any errors before executing the program
    if (krnxErrorReset()) { 
        LOG_INFO("Error reset successfully for controller: {}", cont_no_);
    } 

    if (!setMonitorSpeed(10)) {
        LOG_ERROR("Failed to set monitor speed for RTC mode: {}", cont_no_);
    }

    if (!krnxOldCompClear()) {
        LOG_ERROR("Failed to clear old comp for RTC mode: {}", cont_no_);
    }

    if (!executeOnce("gka_rb_rtc")) {
        LOG_ERROR("Failed to execute program for RTC mode: {}", cont_no_);
        return false;
    }

    switchMode(RtcController); // Switch to RTC controller mode

    LOG_INFO("RTC mode activated successfully for controller: {}", cont_no_);
    return true;
}

bool RobotKrnxDev::setRtcInfo(TKrnxRtcInfo rtc_info)
{
    int ret = krnx_SetRtcInfo(cont_no_, &rtc_info);
    if (ret < 0) {
        LOG_ERROR("Failed to set RTC info: {}", ret);
        return false;
    }

    LOG_INFO("RTC info set successfully for controller: {}", cont_no_);
    return true;
}

bool RobotKrnxDev::krnxErrorReset()
{
    int err_code = 0;
    int ret = krnx_Ereset(cont_no_, robot_no_, &err_code);
    if (ret < 0) {
        LOG_ERROR("Failed to reset error: {} error code {}", ret, err_code);
        return false;
    }

    LOG_INFO("Error reset successfully for controller: {}", cont_no_);
    return true;
}

bool RobotKrnxDev::setMonitorSpeed(float speed)
{
    int err_code = 0;
    int ret = krnx_SetMonSpeed(cont_no_, robot_no_, speed, &err_code);
    if (ret < 0) {
        LOG_ERROR("Failed to set monitor speed: {} error code {}", ret, err_code);
        return false;
    }

    LOG_INFO("Monitor speed set successfully for controller: {}", cont_no_);
    return true;
}

bool RobotKrnxDev::krnxOldCompClear()
{
    int ret = krnx_OldCompClear(cont_no_, robot_no_);
    if (ret < 0) {
        LOG_ERROR("Failed to clear old comp: {}", ret);
        return false;
    }

    LOG_INFO("Old comp cleared successfully for controller: {}", cont_no_);
    return true;
}

bool RobotKrnxDev::krnxHold()
{
    int err_code = 0;
    int ret = krnx_Hold(cont_no_, robot_no_, &err_code);
    if (ret < 0) {
        LOG_ERROR("Failed to hold: {} error code {}", ret, err_code);
        return false;
    }

    LOG_INFO("Hold successfully for controller: {}", cont_no_);
    return true;
}

bool RobotKrnxDev::krnxKill()
{
    int err_code = 0;
    int ret = krnx_Kill(cont_no_, robot_no_, &err_code);
    if (ret < 0) {
        LOG_ERROR("Failed to kill: {} error code {}", ret, err_code);
        return false;
    }

    LOG_INFO("Kill successfully for controller: {}", cont_no_);
    return true;
}

bool RobotKrnxDev::deactivateRtcMode()
{
    if (!krnxHold()) {
        LOG_ERROR("Failed to hold for RTC mode: {}", cont_no_);
        return false;
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));

    if (!krnxKill()) {
        LOG_ERROR("Failed to kill for RTC mode: {}", cont_no_);
        return false;
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Reset any errors before executing the program
    if (krnxErrorReset()) { 
        LOG_INFO("Error reset successfully for controller: {}", cont_no_);
    }

    LOG_INFO("RTC mode deactivated successfully for controller: {}", cont_no_);
    return true;
}

bool RobotKrnxDev::runProgram()
{
    bool ret = motorPower(); // Example: turn on motor power
    if (!ret) {
        LOG_ERROR("Failed to turn on motor power for controller: {}", cont_no_);
        return false;
    }

    ret = switchMode();
    if (!ret) {
        LOG_ERROR("Failed to switch mode for controller: {}", cont_no_);
        return false;
    }

    ret = setSignal(SignalAsProgExecuting, true); // Example: set signal 1 to true
    if (!ret) {
        LOG_ERROR("Failed to set signal for program execution for controller: {}", cont_no_);
        return false;
    }

    ret = activateRtcMode(); // Example: activate RTC mode
    if (!ret) {
        LOG_ERROR("Failed to activate RTC mode for controller: {}", cont_no_);
        return false;
    }



    int rct_sw = 0;
    int  try_read = 0;
    for (try_read = 0; try_read < 10; ++try_read) {
        if (getRtcSwitch(&rct_sw)) {
            if (rct_sw == 1) {
                LOG_INFO("RTC switch is ON for controller: {}", cont_no_);
                break;
            }
        } 
        std::this_thread::sleep_for(std::chrono::milliseconds(500)); // Wait before retrying
    }

    devCallback(); // Start reading data in a separate thread

    return true;
}

bool RobotKrnxDev::getCurMotionDataEx(TKrnxCurMotionDataEx &motion_data)
{
    int ret = krnx_GetCurMotionDataEx(cont_no_, robot_no_, &motion_data);
    if (ret < 0) {
        LOG_ERROR("Failed to get RTC current motion data: {} {}", ret, robot_no_);
        return false;
    }

    return true;
}

void RobotKrnxDev::readRobotData()
{
    if (read_callback_ == nullptr) {
        LOG_WARN("Read callback is not set for controller: {}", cont_no_);
        return ;
    }

    TKrnxCurMotionDataEx motion_data_ex;
    if (!getCurMotionDataEx(motion_data_ex)) {
        LOG_ERROR("Failed to get current motion data: {}", cont_no_);
        return;
    }

    MotionData motion_data;
    if (parseRtcMotionDataEx(motion_data_ex, motion_data)) {
        read_callback_(motion_data);
    }
}

void RobotKrnxDev::devCallback()
{
    std::thread([this]() {
        while (is_opened_) {
            readRobotData();
            std::this_thread::sleep_for(std::chrono::milliseconds(500)); // Sleep for a short duration
        }
    }).detach(); // Detach the thread to run independently

    std::thread([this]() {
        while (is_opened_) {
            moveJoint(delta_angle_.data(), delta_angle_.size());
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Sleep for a short duration
        }
    }).detach(); // Detach the thread to run independently
}

bool RobotKrnxDev::parseRtcMotionDataEx(const TKrnxCurMotionDataEx &motion_data_ex, MotionData &motion_data)
{
    // Parse the RTC motion data from TKrnxCurMotionDataEx to RtcMotionData
    for (int i = 0; i < 6; ++i) {
        motion_data.angles[i] = motion_data_ex.ang[i];
        motion_data.xyzoat[i] = motion_data_ex.xyzoat[i];
        motion_data.xyzptr[i] = motion_data_ex.xyzoat[i]; // Assuming xyzptr is same as xyzoat
    }

    // Additional parsing logic can be added here if needed

    LOG_INFO("Parsed RTC motion data successfully for controller: {}", cont_no_);
    return true;
}

bool RobotKrnxDev::getRtcSwitch(int *rct_sw)
{
    int ret = krnx_GetRtcSwitch(cont_no_, robot_no_, rct_sw);
    if (ret < 0) {
        LOG_ERROR("Failed to get RTC switch: {}", ret);
        return false;
    }

    LOG_INFO("RTC switch retrieved successfully for controller: {}", cont_no_);
    return true;
}
