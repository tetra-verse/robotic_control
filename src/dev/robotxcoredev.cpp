
#include "dev/robotxcoredev.h"

#include <cmath>
#include <string>

#include "log/logging.h"
#include "rokae/data_types.h"
#include "rokae/utility.h"

using namespace rokae;

RobotXCoreDev::RobotXCoreDev(const std::string& remote_ip, const std::string& local_ip)
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
        }
    } catch (const std::exception& e) {
        LOG_ERROR("Exception occurred while connecting to robot: {}", e.what());
        return false;
    }

    setRobotMode();

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
        robot_dev_->setOperateMode(rokae::OperateMode::automatic, ec);
        robot_dev_->setRtNetworkTolerance(20, ec);

        robot_dev_->setMotionControlMode(MotionControlMode::RtCommand, ec);
        robot_dev_->setPowerState(true, ec);
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
    }
}

int RobotXCoreDev::moveLine(float delta, int index)
{
    if (index < 0 || index >= 6) {
        LOG_ERROR("Invalid joint index: {}", index);
        return -1;
    }

    std::error_code ec;
    CartesianPosition start, aux, target;
    auto posture = robot_dev_->posture(rokae::CoordinateType::flangeInBase, ec);
    Utils::postureToTransArray(posture, start.pos);
    // Utils::arrayToTransMatrix(start.pos, rot_start, trans_start);

    return 0;
}

int RobotXCoreDev::moveForward(float delta) { return moveLine(delta, 0); }

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
