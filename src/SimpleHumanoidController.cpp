#include "SimpleHumanoidController.h"

#include <mc_rbdyn/RobotFrame.h>
#include <mc_rtc/logging.h>
#include <mc_tasks/MetaTaskLoader.h>

SimpleHumanoidController::SimpleHumanoidController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration &config)
    : mc_control::fsm::Controller(rm, dt, config)
{
  mc_rtc::log::success("SimpleHumanoidController init done");
}

bool SimpleHumanoidController::run()
{
  return mc_control::fsm::Controller::run();
}

void SimpleHumanoidController::reset(const mc_control::ControllerResetData &reset_data)
{

  // This is not necessary. Attempting to save true original position and load it to config file.
  // auto saveHandPose = [this](const std::string &name)
  // {
  //   auto pose = robot().bodyPosW(name);

  //   // Position (Vector3d -> std::vector<double>)
  //   Eigen::Vector3d pos = pose.translation();
  //   config_.add(name + "InitPos", std::vector<double>{pos.x(), pos.y(), pos.z()});

  //   // Orientation (Matrix3d -> Quaterniond -> std::vector<double>)
  //   Eigen::Quaterniond ori(pose.rotation());
  //   config_.add(name + "InitOri", std::vector<double>{ori.x(), ori.y(), ori.z(), ori.w()});

  //   // Debug logging
  //   // mc_rtc::log::info("{}InitPos = {}", name, std::vector<double>{pos.x(), pos.y(), pos.z()});
  //   // mc_rtc::log::info("{}InitOri = {}", name, std::vector<double>{ori.x(), ori.y(), ori.z(), ori.w()});
  //   mc_rtc::log::info("{}InitPos = {}", name, pose.translation());
  //   mc_rtc::log::info("{}InitPos = {}", name, pose.rotation());
  // };

  // saveHandPose("l_wrist");
  // saveHandPose("r_wrist");
  // ----------------------------------------------------------------------------------------------

  mc_control::fsm::Controller::reset(reset_data);
}
