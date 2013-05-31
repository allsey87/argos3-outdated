
/**
 * @file <argos3/testing/dyn3d_testbench/robot_joints_controller.cpp>
 *
 * @author Michael Allwright <allsey87@gmail.com>
 */
 
#include "robot_joints_controller.h"
#include <argos3/core/utility/logging/argos_log.h>

/****************************************/
/****************************************/

CRobotJointsController::CRobotJointsController() {
}

/****************************************/
/****************************************/

CRobotJointsController::~CRobotJointsController() {
}

/****************************************/
/****************************************/

void CRobotJointsController::Init(TConfigurationNode& t_tree) {
   m_pcJoints = GetActuator<CCI_RobotJointsActuator>("joints");
   m_pcJointActuator = &m_pcJoints->GetJointActuator("joint_1", CCI_RobotJointsActuator::ANGULAR_X);
   m_pcJointActuator->SetTargetVelocity(5);
   m_unCount = 0;
}

/****************************************/
/****************************************/

void CRobotJointsController::Reset() {
}

/****************************************/
/****************************************/

void CRobotJointsController::Destroy() {
}

/****************************************/
/****************************************/

void CRobotJointsController::ControlStep() {
   m_unCount++;
   if(m_unCount % 100 == 0) {
      m_pcJointActuator->SetTargetVelocity(-m_pcJointActuator->GetTargetVelocity());
   }
   
}

/****************************************/
/****************************************/

REGISTER_CONTROLLER(CRobotJointsController, "robot_joints_controller");
