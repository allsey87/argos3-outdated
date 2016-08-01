
/**
 * @file <argos3/testing/bebot_controller/src/bebot_controller.cpp>
 *
 * @author Michael Allwright <allsey87@gmail.com>
 */

#include "bebot_controller.h"
#include <argos3/core/utility/logging/argos_log.h>

/****************************************/
/****************************************/

CBeBotController::CBeBotController() {
}

/****************************************/
/****************************************/

CBeBotController::~CBeBotController() {
}

/****************************************/
/****************************************/

void CBeBotController::Init(TConfigurationNode& t_tree) {
   /* sensors */
   m_pcRadiosSensor = GetSensor<CCI_PrototypeRadiosSensor>("radios");
   m_pcJointsSensor = GetSensor<CCI_PrototypeJointsSensor>("joints");   
   m_pcProximitySensor = GetSensor<CCI_PrototypeProximitySensor>("prototype_proximity");
   /* actuators */
   m_pcJointsActuator = GetActuator<CCI_PrototypeJointsActuator>("joints");
   m_pcRadiosActuator = GetActuator<CCI_PrototypeRadiosActuator>("radios");
   m_pcElectromagnets = GetActuator<CCI_PrototypeElectromagnetsActuator>("electromagnets");

   /* fetch joint actuators */   
   m_pcLiftActuatorJoint = &(m_pcJointsActuator->GetJointActuator("lift-fixture:vertical-link", CCI_PrototypeJointsActuator::LINEAR_Z));
   m_pcFrontLeftWheelJoint = &(m_pcJointsActuator->GetJointActuator("wheel-front-left:lower-chassis", CCI_PrototypeJointsActuator::ANGULAR_Y));
   m_pcFrontRightWheelJoint = &(m_pcJointsActuator->GetJointActuator("wheel-front-right:lower-chassis", CCI_PrototypeJointsActuator::ANGULAR_Y));
   m_pcRearLeftWheelJoint = &(m_pcJointsActuator->GetJointActuator("wheel-rear-left:lower-chassis", CCI_PrototypeJointsActuator::ANGULAR_Y));
   m_pcRearRightWheelJoint = &(m_pcJointsActuator->GetJointActuator("wheel-rear-right:lower-chassis", CCI_PrototypeJointsActuator::ANGULAR_Y));

   Reset();
}

/****************************************/
/****************************************/

void CBeBotController::Reset() {
   //m_pcLiftActuatorJoint->SetTargetVelocity(0.2);
   //SetTargetVelocity(-2,2);
}

/****************************************/
/****************************************/

void CBeBotController::Destroy() {
}

/****************************************/
/****************************************/

void CBeBotController::ControlStep() {
 
   // simulate the lift actuator
   // read sensors
   // update data structures
   // step state machine
   // write back to actuators
}

/****************************************/
/****************************************/

void CBeBotController::SetTargetVelocity(double f_left, double f_right) {
   m_pcFrontLeftWheelJoint->SetTargetVelocity(f_left);
   m_pcFrontRightWheelJoint->SetTargetVelocity(f_right);
   m_pcRearLeftWheelJoint->SetTargetVelocity(f_left);
   m_pcRearRightWheelJoint->SetTargetVelocity(f_right);
}

/****************************************/
/****************************************/

void CBeBotController::SetElectromagnetCurrent(double f_value) {
   for(auto& t_configuration : m_pcElectromagnets->GetConfigurations()) {
      t_configuration.Current = f_value;
   }
}

/****************************************/
/****************************************/

REGISTER_CONTROLLER(CBeBotController, "bebot_controller");
