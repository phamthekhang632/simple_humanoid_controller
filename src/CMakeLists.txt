set(controller_SRC
  SimpleHumanoidController.cpp
)

set(controller_HDR
  SimpleHumanoidController.h
)

add_controller(${PROJECT_NAME} ${controller_SRC} ${controller_HDR})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DSimpleHumanoidController_EXPORTS")

