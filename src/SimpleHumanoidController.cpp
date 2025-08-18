#include "SimpleHumanoidController.h"

#include <mc_rbdyn/RobotFrame.h>
#include <mc_rtc/logging.h>
#include <mc_tasks/MetaTaskLoader.h>

#include <chrono>

SimpleHumanoidController::SimpleHumanoidController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration &config)
    : mc_control::MCController(rm, dt)
{
  config_.load(config);
  solver().addConstraintSet(contactConstraint);
  solver().addConstraintSet(dynamicsConstraint);
  solver().addTask(postureTask);

  // CoM
  addContact({robot().name(), "ground", "LeftFoot", "AllGround"});
  addContact({robot().name(), "ground", "RightFoot", "AllGround"});
  postureTask->stiffness(1);

  // End Effector
  leftHandTask_ = mc_tasks::MetaTaskLoader::load<mc_tasks::EndEffectorTask>(solver(), "extensions/simple_humanoid_controller/task.yaml");
  solver().addTask(leftHandTask_);

  // End effectors
  leftHandTask_ = mc_tasks::MetaTaskLoader::load<mc_tasks::EndEffectorTask>(
      solver(),
      "extensions/simple_humanoid_controller/etc/left_task.yaml");
  solver().addTask(leftHandTask_);
  rightHandTask_ = mc_tasks::MetaTaskLoader::load<mc_tasks::EndEffectorTask>(
      solver(),
      "extensions/simple_humanoid_controller/etc/right_task.yaml");
  solver().addTask(rightHandTask_);

  // Save initial poses
  leftHandInitPose_ = leftHandTask_->get_ef_pose();
  rightHandInitPose_ = rightHandTask_->get_ef_pose();

  stateStartTime_ = std::chrono::steady_clock::now();

  mc_rtc::log::success("SimpleHumanoidController init done");
}

bool SimpleHumanoidController::run()
{
  auto now = std::chrono::steady_clock::now();
  double elapsed = std::chrono::duration<double>(now - stateStartTime_).count();

  if (elapsed > stateDuration_)
  {
    switchState();
    stateStartTime_ = now;
  }

  return mc_control::MCController::run();
}

void SimpleHumanoidController::reset(const mc_control::ControllerResetData &reset_data)
{
  mc_control::MCController::reset(reset_data);
}

void SimpleHumanoidController::switchState()
{
  // Define target positions
  Eigen::Vector3d leftTarget{0.5, 0.25, 1.1};
  Eigen::Vector3d rightTarget{0.5, -0.25, 1.1};
  Eigen::Quaterniond quat(0.7, 0, 0.7, 0); // w,x,y,z
  quat.normalize();

  switch (currentState_)
  {
  case HandState::LEFT_FORWARD:
    leftHandTask_->set_ef_pose({quat, leftTarget});
    currentState_ = HandState::LEFT_BACK;
    break;

  case HandState::LEFT_BACK:
    leftHandTask_->set_ef_pose(leftHandInitPose_);
    currentState_ = HandState::RIGHT_FORWARD;
    break;

  case HandState::RIGHT_FORWARD:
    rightHandTask_->set_ef_pose({quat, rightTarget});
    currentState_ = HandState::RIGHT_BACK;
    break;

  case HandState::RIGHT_BACK:
    rightHandTask_->set_ef_pose(rightHandInitPose_);
    currentState_ = HandState::BOTH_FORWARD;
    break;

  case HandState::BOTH_FORWARD:
    leftHandTask_->set_ef_pose({quat, leftTarget});
    rightHandTask_->set_ef_pose({quat, rightTarget});
    currentState_ = HandState::BOTH_BACK;
    break;

  case HandState::BOTH_BACK:
    leftHandTask_->set_ef_pose(leftHandInitPose_);
    rightHandTask_->set_ef_pose(rightHandInitPose_);
    currentState_ = HandState::LEFT_FORWARD;
    break;
  }
}

CONTROLLER_CONSTRUCTOR("SimpleHumanoidController", SimpleHumanoidController)