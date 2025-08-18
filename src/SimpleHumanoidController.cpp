#include "SimpleHumanoidController.h"

#include <mc_rbdyn/RobotFrame.h>
#include <mc_rtc/logging.h>
#include <mc_tasks/MetaTaskLoader.h>

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

  // End effectors
  leftHandTask_ = mc_tasks::MetaTaskLoader::load<mc_tasks::EndEffectorTask>(
      solver(),
      "extensions/simple_humanoid_controller/etc/left_task.yaml");
  solver().addTask(leftHandTask_);
  rightHandTask_ = mc_tasks::MetaTaskLoader::load<mc_tasks::EndEffectorTask>(
      solver(),
      "extensions/simple_humanoid_controller/etc/right_task.yaml");
  solver().addTask(rightHandTask_);

  // Save target forward and backward poses
  leftForwardPose_ = leftHandTask_->get_ef_pose();
  rightForwardPose_ = rightHandTask_->get_ef_pose();
  leftHandInitPose_ = robot().bodyPosW("l_wrist");
  rightHandInitPose_ = robot().bodyPosW("r_wrist");

  mc_rtc::log::success("SimpleHumanoidController init done");
}

bool SimpleHumanoidController::run()
{
  auto leftError = leftHandTask_->eval();
  auto rightError = rightHandTask_->eval();

  bool leftReached = leftError.norm() < 0.02;
  bool rightReached = rightError.norm() < 0.02;

  if (leftReached && rightReached)
  {
    switchState();
  }

  return mc_control::MCController::run();
}

void SimpleHumanoidController::reset(const mc_control::ControllerResetData &reset_data)
{
  leftHandTask_->reset();
  rightHandTask_->reset();
  currentState_ = HandState::LEFT_FORWARD;

  mc_control::MCController::reset(reset_data);
}

void SimpleHumanoidController::switchState()
{
  switch (currentState_)
  {
  case HandState::LEFT_FORWARD:
    leftHandTask_->set_ef_pose(leftForwardPose_);
    currentState_ = HandState::LEFT_BACK;
    break;

  case HandState::LEFT_BACK:
    leftHandTask_->set_ef_pose(leftHandInitPose_);
    currentState_ = HandState::RIGHT_FORWARD;
    break;

  case HandState::RIGHT_FORWARD:
    rightHandTask_->set_ef_pose(rightForwardPose_);
    currentState_ = HandState::RIGHT_BACK;
    break;

  case HandState::RIGHT_BACK:
    rightHandTask_->set_ef_pose(rightHandInitPose_);
    currentState_ = HandState::BOTH_FORWARD;
    break;

  case HandState::BOTH_FORWARD:
    leftHandTask_->set_ef_pose(leftForwardPose_);
    rightHandTask_->set_ef_pose(rightForwardPose_);
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