
/**
 * @file <argos3/testing/bebot_controller/src/bebot_controller.cpp>
 *
 * @author Michael Allwright <allsey87@gmail.com>
 */

#include "bebot_controller.h"
#include <argos3/core/utility/logging/argos_log.h>
#include <iomanip>

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
   m_pcCamerasSensor = GetSensor<CCI_CamerasSensor>("cameras");
   /* camera sensor algorithms */   
   m_pcLEDDetectorAlgorithm = m_pcCamerasSensor->GetAlgorithm<CCI_CamerasSensorLEDDetectorAlgorithm>("front_camera","led_detector");
   m_pcTagDetectorAlgorithm = m_pcCamerasSensor->GetAlgorithm<CCI_CamerasSensorTagDetectorAlgorithm>("front_camera","tag_detector");
   /* actuators */
   m_pcJointsActuator = GetActuator<CCI_PrototypeJointsActuator>("joints");
   m_pcRadiosActuator = GetActuator<CCI_PrototypeRadiosActuator>("radios");
   m_pcElectromagnets = GetActuator<CCI_PrototypeElectromagnetsActuator>("electromagnets");
   /* wheels */   
   m_pcFrontLeftWheelJoint = &(m_pcJointsActuator->GetJointActuator("wheel-front-left:lower-chassis", CCI_PrototypeJointsActuator::ANGULAR_Y));
   m_pcFrontRightWheelJoint = &(m_pcJointsActuator->GetJointActuator("wheel-front-right:lower-chassis", CCI_PrototypeJointsActuator::ANGULAR_Y));
   m_pcRearLeftWheelJoint = &(m_pcJointsActuator->GetJointActuator("wheel-rear-left:lower-chassis", CCI_PrototypeJointsActuator::ANGULAR_Y));
   m_pcRearRightWheelJoint = &(m_pcJointsActuator->GetJointActuator("wheel-rear-right:lower-chassis", CCI_PrototypeJointsActuator::ANGULAR_Y));
   /* lift actuator system */
   m_pcLiftActuatorSystemController = new CLiftActuatorSystemController(
      m_pcJointsSensor->GetJointSensor("lift-fixture:vertical-link"),
      &(m_pcJointsActuator->GetJointActuator("lift-fixture:vertical-link", CCI_PrototypeJointsActuator::LINEAR_Z))
   );
   /* issue reset */
   Reset();
}

/****************************************/
/****************************************/

void CBeBotController::Reset() {
   m_pcLiftActuatorSystemController->Reset();
   SetElectromagnetCurrent(1.0);
   //SetTargetVelocity(-2,2);
   //m_pcFrontLeftWheelJoint->SetTargetVelocity(5);
   //m_pcFrontRightWheelJoint->SetTargetVelocity(5);

}

/****************************************/
/****************************************/

void CBeBotController::Destroy() {
   delete m_pcLiftActuatorSystemController;
}

/****************************************/
/****************************************/

void CBeBotController::ControlStep() {
   m_pcLiftActuatorSystemController->ControlStep();
   
   /*
   std::cout << "detected: " 
             << m_pcLEDDetectorAlgorithm->GetReadings().size()
             << " leds, "
             << m_pcTagDetectorAlgorithm->GetReadings().size()
             << " tags"
             << std::endl;
   */
   for(auto& t_reading : m_pcTagDetectorAlgorithm->GetReadings()) {
      CRadians pcEulers[3];
      t_reading.Orientation.ToEulerAngles(pcEulers[0], pcEulers[1], pcEulers[2]);
      std::cout << std::fixed << std::setprecision(1) << t_reading.Payload << " [" 
         << ToDegrees(pcEulers[0]).GetValue() << ", "
         << ToDegrees(pcEulers[1]).GetValue() << ", "
         << ToDegrees(pcEulers[2]).GetValue() << "]"
         << std::endl;
/*
      std::cout << std::fixed << std::setprecision(3) << t_reading.Payload << " [" 
         << t_reading.Position.GetX() << ", "
         << t_reading.Position.GetY() << ", "
         << t_reading.Position.GetZ() << "]"
         << std::endl;
*/
   }

   // simulate the lift actuator
   // read sensors
   // update data structures
   // step state machine
   // write back to actuators
}

/****************************************/
/****************************************/

void CBeBotController::SetTargetVelocity(Real f_left, Real f_right) {
   m_pcFrontLeftWheelJoint->SetTargetVelocity(f_left);
   m_pcFrontRightWheelJoint->SetTargetVelocity(f_right);
   m_pcRearLeftWheelJoint->SetTargetVelocity(f_left);
   m_pcRearRightWheelJoint->SetTargetVelocity(f_right);
}

/****************************************/
/****************************************/

void CBeBotController::SetElectromagnetCurrent(Real f_value) {
   for(auto& t_configuration : m_pcElectromagnets->GetConfigurations()) {
      t_configuration.Current = f_value;
   }
}

/****************************************/
/****************************************/

REGISTER_CONTROLLER(CBeBotController, "bebot_controller");
