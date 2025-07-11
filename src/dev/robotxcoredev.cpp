
#include "dev/robotxcoredev.h"

#include <string>
#include <cmath>
#include <corecrt_math_defines.h>

#include "rokae/data_types.h"
#include "rokae/utility.h"
#include "log/logging.h"

using namespace rokae;

RobotXCoreDev::RobotXCoreDev(const std::string &remote_ip, const std::string& local_ip)
    : remote_ip_(remote_ip), local_ip_(local_ip)
{
    
}

RobotXCoreDev::~RobotXCoreDev()
{
    disconnect();

    if (robot_dev_ != nullptr) {
        delete robot_dev_;
        robot_dev_ = nullptr;
    }
}

bool RobotXCoreDev::connect()
{
    try {
        robot_dev_ = new rokae::xMateRobot(remote_ip_, local_ip_);
        if (robot_dev_ == nullptr) {
            LOG_ERROR("Failed to create robot device {}", remote_ip_);
            return false;
        }

        setRobotMode();
        setRobotReceiveState();
    }
    catch (const std::exception &e) {
        LOG_ERROR("Exception occurred while connecting to robot: {}", e.what());
        return false;
    }

    // Create the robot device
    LOG_INFO("Creating robot device with remote IP: {}, local IP: {}", remote_ip_, local_ip_);

    return true;
}

bool RobotXCoreDev::disconnect()
{
    if (robot_dev_ != nullptr) {
        error_code ec;
        robot_dev_->disconnectFromRobot(ec);
        LOG_INFO("Robot device disconnected successfully");
    }

    return true;
}

void RobotXCoreDev::setRobotMode()
{
    LOG_INFO("Setting robot mode");

    try {
        error_code ec;
        robot_dev_->setOperateMode(rokae::OperateMode::manual,ec);
        robot_dev_->setRtNetworkTolerance(20, ec);

        robot_dev_->setMotionControlMode(MotionControlMode::RtCommand, ec);
        robot_dev_->setPowerState(true, ec);
    }
    catch(const std::exception& e) {
        std::cerr << e.what() << '\n';
    }
}

void RobotXCoreDev::setRobotReceiveState()
{
    LOG_INFO("Setting robot receive state");

    error_code ec;
    std::shared_ptr<RtMotionControlCobot<6>> rtCon;
    rtCon = robot_dev_->getRtMotionController().lock();
    robot_dev_->startReceiveRobotState(std::chrono::milliseconds(20), {RtSupportedFields::tcpPoseAbc_m, RtSupportedFields::tau_m, RtSupportedFields::jointPos_m});

    std::function<JointPosition(void)> callback = [&, rtCon]() {
        JointPosition cmd(7);

        MotionData data;
        if (read_callback_ && readRobotData(data)) {
            read_callback_(data);
        }

        std::array<double,6> jntPos{};
        robot_dev_->getStateData(RtSupportedFields::jointPos_m, jntPos);
        for(unsigned i = 0; i < cmd.joints.size() && i < sizeof(delta_angle_) / sizeof(float); ++i) {
            cmd.joints[i] = jntPos[i] + delta_angle_[i];
        }

        return cmd;
    };

    rtCon->setControlLoop(callback, 0, true);
    rtCon->startLoop(true);
}

bool RobotXCoreDev::readRobotData(MotionData &data)
{
    error_code ec;
    auto tcp_xyzabc = robot_dev_->posture(CoordinateType::endInRef, ec);
    auto flan_cart = robot_dev_->cartPosture(CoordinateType::flangeInBase, ec);
    robot_dev_->baseFrame(ec); // 基坐标系
    // printf( "末端相对外部参考坐标系位姿", tcp_xyzabc);
    // printf("法兰相对基坐标系 -", flan_cart);

    std::array<double,6> jntPos{};
    robot_dev_->getStateData(RtSupportedFields::jointPos_m, jntPos);
    return true;
}

int RobotXCoreDev::moveDegree(float delta, int index)
{
    if (index < 0 || index >= 3) {
        LOG_ERROR("Invalid joint index: {}", index);
        return -1;
    }

    std::error_code ec;
    CartesianPosition start, aux, target;
    Eigen::Matrix3d rot_start;
    Eigen::Vector3d trans_start, trans_aux, trans_end;
    auto rtCon = robot_dev_->getRtMotionController().lock();

    auto posture = robot_dev_->posture(rokae::CoordinateType::flangeInBase, ec);
    Utils::postureToTransArray(posture, start.pos);
    Utils::arrayToTransMatrix(start.pos, rot_start, trans_start);

    trans_end = trans_start;
    trans_aux = trans_start;
    trans_aux[index] += delta / 2;
    trans_end[index] += delta;

    Utils::transMatrixToArray(rot_start, trans_aux, aux.pos);
    Utils::transMatrixToArray(rot_start, trans_end, target.pos);

    rtCon->MoveC(speed_, start, aux, target);

    return 0;
}

int RobotXCoreDev::moveLine(float delta, int index)
{
    if (index < 0 || index >= 3) {
        LOG_ERROR("Invalid joint index: {}", index);
        return -1;
    }

    std::error_code ec;
    CartesianPosition start, target;
    Eigen::Matrix3d rot_start;
    Eigen::Vector3d trans_start, trans_end;
    auto rtCon = robot_dev_->getRtMotionController().lock();

    auto posture = robot_dev_->posture(rokae::CoordinateType::flangeInBase, ec);
    Utils::postureToTransArray(posture, start.pos);
    Utils::arrayToTransMatrix(start.pos, rot_start, trans_start);

    trans_end = trans_start;
    trans_end[index] += delta;

    Utils::transMatrixToArray(rot_start, trans_end, target.pos);
    rtCon->MoveL(speed_, start, target);

    return 0;
}

int RobotXCoreDev::moveForward(float delta)
{
    return moveLine(delta, 0);
}

int RobotXCoreDev::moveBackward(float delta)
{
    return moveLine(-delta, 0);
}

int RobotXCoreDev::moveLeftward(float delta)
{
    return moveLine(delta, 1);
}

int RobotXCoreDev::moveRightward(float delta)
{
    return moveLine(-delta, 1);
}

int RobotXCoreDev::moveUpward(float delta)
{
    return moveLine(delta, 2);
}

int RobotXCoreDev::moveDownward(float delta)
{
    return moveLine(-delta, 2);
}

int RobotXCoreDev::moveJoint(float delta, int index)
{
    if (index < 0 || index >= 6) {
        LOG_ERROR("Invalid joint index: {}", index);
        return -1;
    }

    std::error_code ec;
    std::array<double, 6> joint_pos = robot_dev_->jointPos(ec);
    if (!ec) {
        LOG_ERROR("Failed to get joint position: {}", ec.message());
        return -1;
    }

    std::array<double, 6> target_pos = joint_pos;

    target_pos[index] += delta;
    auto rtCon = robot_dev_->getRtMotionController().lock();
    rtCon->MoveJ(speed_, joint_pos, target_pos);

    return 0;
}

int RobotXCoreDev::moveJoint(float *delta, int size)
{
    return 0;
}

int RobotXCoreDev::moveSpeed(float *delta, int size)
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


int RobotXCoreDev::moveUp(float delta)
{
    return moveDegree(delta, 0);
}

int RobotXCoreDev::moveDown(float delta)
{
    return moveDegree(-delta, 0);
}

int RobotXCoreDev::moveLeft(float delta)
{
    return moveDegree(delta, 1);
}

int RobotXCoreDev::moveRight(float delta)
{
    return moveDegree(-delta, 1);
}

int RobotXCoreDev::moveRollClockwise(float delta)
{
    return moveDegree(delta, 2);
}

int RobotXCoreDev::moveRollCounterClockwise(float delta)
{
    return moveDegree(-delta, 2);
}

void RobotXCoreDev::setReadCallback(ReadCallback callback)
{
    read_callback_ = callback;
}
