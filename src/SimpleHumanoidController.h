#pragma once

#include <mc_control/mc_controller.h>
#include <mc_tasks/CoMTask.h>
#include <mc_tasks/EndEffectorTask.h>
#include <mc_tasks/LookAtTask.h>

#include "api.h"

struct SimpleHumanoidController_DLLAPI SimpleHumanoidController : public mc_control::MCController
{
  SimpleHumanoidController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration &config);

  bool run() override;
  void reset(const mc_control::ControllerResetData &reset_data) override;

private:
  mc_rtc::Configuration config_{};

  std::shared_ptr<mc_tasks::EndEffectorTask> leftHandTask_;
  std::shared_ptr<mc_tasks::EndEffectorTask> rightHandTask_;
  Eigen::Vector3d gazeVector{1, 0, 0};
  std::string lookingTarget{"l_wrist"};
  std::shared_ptr<mc_tasks::LookAtTask> lookAtTask_;

  // Timing and state
  enum class HandState
  {
    LEFT_FORWARD,
    LEFT_BACK,
    RIGHT_FORWARD,
    RIGHT_BACK,
    BOTH_FORWARD,
    BOTH_BACK
  };
  HandState currentState_ = HandState::LEFT_FORWARD;

  // Store target poses
  sva::PTransformd leftForwardPose_;
  sva::PTransformd rightForwardPose_;
  sva::PTransformd leftHandInitPose_;
  sva::PTransformd rightHandInitPose_;

  void switchState();
};