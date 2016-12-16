
/**
 * @file <argos3/testing/bebot_controller/src/bebot_controller.cpp>
 *
 * @author Michael Allwright <allsey87@gmail.com>
 */

#include "bebot_controller.h"
#include <argos3/core/utility/logging/argos_log.h>
#include <iomanip>

namespace argos {

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
      m_pcLEDDetectorAlgorithm = m_pcCamerasSensor->GetAlgorithm<CCI_CamerasSensorLEDDetectorAlgorithm>("duovero_camera","led_detector");
      m_pcTagDetectorAlgorithm = m_pcCamerasSensor->GetAlgorithm<CCI_CamerasSensorTagDetectorAlgorithm>("duovero_camera","tag_detector");
      /* init modules */
      m_pcBlockDetector = new CBlockDetector;
      m_pcBlockDetector->SetCameraMatrix(m_pcCamerasSensor->GetCameraMatrix("duovero_camera"));
      m_pcBlockDetector->SetDistortionParameters(m_pcCamerasSensor->GetDistortionParameters("duovero_camera"));

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
      SetElectromagnetCurrent(0.0);
      SetTargetVelocity(0.0,0.0);
      /* delete data structs */
      delete m_psSensorData;
      delete m_psActuatorData;

      /* recreate data structs */
      m_psSensorData = new CBlockDemo::SSensorData;
      m_psActuatorData = new CBlockDemo::SActuatorData;

      /* Update the global data struct pointers */
      //Data.Sensors = m_psSensorData;
      //Data.Actuators = m_psActuatorData;
      /* Create lists for blocks, targets, and structures */
      // TODO attached block lists to sensor data


      m_tDetectedBlockList.clear();

      m_tpLastReset = std::chrono::steady_clock::now();
   }

   /****************************************/
   /****************************************/

   void CBeBotController::Destroy() {
      delete m_pcLiftActuatorSystemController;
      delete m_psSensorData;
      delete m_psActuatorData;
      delete m_pcBlockDetector;
   }

   /****************************************/
   /****************************************/

   void CBeBotController::ControlStep() {
      // step the las controller
      m_pcLiftActuatorSystemController->ControlStep();
    
      /*
      // send radio message  
      CCI_PrototypeRadiosActuator::SConfiguration* psConfig = 
         &m_pcRadiosActuator->GetConfigurations()[0];
    
      psConfig->TxData.clear();
      psConfig->TxData.emplace_back();
      psConfig->TxData.back() << '1';
      */

      /* clear out the old detections */
      m_tDetectedBlockList.clear();

      /* create a time point for the current detections */
      std::chrono::time_point<std::chrono::steady_clock> tpControlStep =
         std::chrono::steady_clock::now();
      
      /* do block, target and structure detection */
      const CCI_CamerasSensorTagDetectorAlgorithm::SReading::TList& tTagReadings =
         m_pcTagDetectorAlgorithm->GetReadings();
      const CCI_CamerasSensorLEDDetectorAlgorithm::SReading::TList& tLedReadings =
         m_pcLEDDetectorAlgorithm->GetReadings();

      m_pcBlockDetector->Detect(tTagReadings, tLedReadings, m_tDetectedBlockList);

      /* TODO:
         1. simulate the lift actuator control loop - done
         2. read sensors
         3. update data structures
         4. step state machine
         5. write back to actuators
      */
   }

   /****************************************/
   /****************************************/

   void CBeBotController::SetTargetVelocity(Real f_left, Real f_right) {
      m_pcFrontLeftWheelJoint->SetTargetVelocity(-f_left);
      m_pcFrontRightWheelJoint->SetTargetVelocity(-f_right);
      m_pcRearLeftWheelJoint->SetTargetVelocity(-f_left);
      m_pcRearRightWheelJoint->SetTargetVelocity(-f_right);
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

   /****************************************/
   /****************************************/
}


