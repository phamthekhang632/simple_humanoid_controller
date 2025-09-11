#include <mc_control/fsm/Controller.h>

#include "SimpleHumanoidController_Initial.h"
#include "../SimpleHumanoidController.h"

void SimpleHumanoidController_Initial::configure(const mc_rtc::Configuration &config)
{
}

void SimpleHumanoidController_Initial::start(mc_control::fsm::Controller &ctl_)
{
  ctl_.gui()->addElement(
      {},
      mc_rtc::gui::Button("Start Moving", [this]()
                          { startMoving_ = true; }));
}

bool SimpleHumanoidController_Initial::run(mc_control::fsm::Controller &ctl_)
{
  if (startMoving_)
  {
    output("StartMoving");
    return true;
  }
  return false;
}

void SimpleHumanoidController_Initial::teardown(mc_control::fsm::Controller &ctl_)
{
  ctl_.gui()->removeElement({}, "Start Moving");
}

EXPORT_SINGLE_STATE("SimpleHumanoidController_Initial", SimpleHumanoidController_Initial);