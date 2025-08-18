#include "SimpleHumanoidController.h"

#include <mc_rbdyn/RobotFrame.h>
#include <mc_rtc/logging.h>
#include <mc_tasks/MetaTaskLoader.h>

SimpleHumanoidController::SimpleHumanoidController(
    mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration &config)
    : mc_control::MCController(rm, dt)
{
  config_.load(config);

  solver().addConstraintSet(contactConstraint);
  solver().addConstraintSet(selfCollisionConstraint);
  solver().addConstraintSet(dynamicsConstraint);
  solver().addConstraintSet(compoundJointConstraint);
  solver().addTask(postureTask);
  postureTask->stiffness(10);

  // Enforce left and right foot contacts
  addContact({robot().name(), "ground", "LeftFoot", "AllGround"});
  addContact({robot().name(), "ground", "RightFoot", "AllGround"});

  // End effectors tasks to move hands
  leftHandTask_ = mc_tasks::MetaTaskLoader::load<mc_tasks::EndEffectorTask>(
      solver(), config("LeftHandTask"));
  solver().addTask(leftHandTask_);
  rightHandTask_ = mc_tasks::MetaTaskLoader::load<mc_tasks::EndEffectorTask>(
      solver(), config("RightHandTask"));
  solver().addTask(rightHandTask_);

  // Save target forward and backward poses
  leftForwardPose_ = leftHandTask_->get_ef_pose();
  rightForwardPose_ = rightHandTask_->get_ef_pose();
  leftHandInitPose_ = robot().bodyPosW("l_wrist");
  rightHandInitPose_ = robot().bodyPosW("r_wrist");

  // Looking at hand
  lookAtTask_ = std::make_shared<mc_tasks::LookAtTask>(
      "NECK_P_S",
      gazeVector,
      robot().bodyPosW(lookingTarget).translation(),
      robots(),
      0);
  lookAtTask_->selectActiveJoints(solver(), {"NECK_P", "NECK_R", "NECK_Y"});
  solver().addTask(lookAtTask_);

  mc_rtc::log::success("SimpleHumanoidController init done");
}

bool SimpleHumanoidController::run()
{
  float leftError = leftHandTask_->eval().norm();
  float rightError = rightHandTask_->eval().norm();
  if (leftError < 0.02 && rightError < 0.02)
  {
    switchState();
  }

  updateLookingTask(leftError, rightError);

  return mc_control::MCController::run();
}

void SimpleHumanoidController::reset(const mc_control::ControllerResetData &reset_data)
{
  leftHandTask_->reset();
  rightHandTask_->reset();
  currentState_ = HandState::LEFT_FORWARD;

  lookAtTask_->reset();

  mc_control::MCController::reset(reset_data);
}

void SimpleHumanoidController::switchState()
{
  /*
  Switches the state of the robot to follow task in desired sequence.
  */

  switch (currentState_)
  {
  case HandState::LEFT_FORWARD:
    leftHandTask_->set_ef_pose(leftForwardPose_);
    lookingTarget = "l_wrist";
    currentState_ = HandState::LEFT_BACK;
    break;

  case HandState::LEFT_BACK:
    leftHandTask_->set_ef_pose(leftHandInitPose_);
    currentState_ = HandState::RIGHT_FORWARD;
    break;

  case HandState::RIGHT_FORWARD:
    rightHandTask_->set_ef_pose(rightForwardPose_);
    lookingTarget = "r_wrist";
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

void SimpleHumanoidController::updateLookingTask(float leftError, float rightError)
{
  /*
  The looking task is updated based on the current state of the hands.
  When one hand is moving, we look at the moving hand.
  When both hands are moving back, we look straight ahead.
  */

  // The currentState_ checking is a bit counter-intuitive
  // Since the currentState_ is update as soon as switchState() is called
  // We need to check the with the state one step into the future
  if (currentState_ == HandState::BOTH_BACK || currentState_ == HandState::LEFT_FORWARD)
  {
    lookAtTask_->target(Eigen::Vector3d(1000, 0, 0)); // Looking far ahead
  }
  else
  {
    // Stop looking at the hand when moving back to original position
    // This is to anoid having the pictch angle too low
    if ((currentState_ == HandState::RIGHT_FORWARD && leftError < 0.2) ||
        (currentState_ == HandState::BOTH_FORWARD && rightError < 0.2))
    {
      lookAtTask_->target(Eigen::Vector3d(1000, 0, 0));
    }
    else
    {
      lookAtTask_->target(robot().bodyPosW(lookingTarget).translation());
    }
  }
}

CONTROLLER_CONSTRUCTOR("SimpleHumanoidController", SimpleHumanoidController)