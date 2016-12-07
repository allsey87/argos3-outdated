
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
   m_pcLEDDetectorAlgorithm = m_pcCamerasSensor->GetAlgorithm<CCI_CamerasSensorLEDDetectorAlgorithm>("duovero_camera","led_detector");
   m_pcTagDetectorAlgorithm = m_pcCamerasSensor->GetAlgorithm<CCI_CamerasSensorTagDetectorAlgorithm>("duovero_camera","tag_detector");
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
   /* Get pointer to the LED index - note: hard coded amongst other design issues */
   m_pcLEDIndex = &(CSimulator::GetInstance().GetMedium<CLEDMedium>("leds").GetIndex());
   /* issue reset */
   Reset();
}

/****************************************/
/****************************************/

void CBeBotController::Reset() {
   m_pcLiftActuatorSystemController->Reset();
   SetElectromagnetCurrent(0.0);
   SetTargetVelocity(0.0,0.0);
   /* delete the old data structs */
   delete m_psSensorData;
   delete m_psActuatorData;
   /* create new data structs */
   m_psSensorData = new CBlockDemo::SSensorData;
   m_psActuatorData = new CBlockDemo::SActuatorData;
   /* Update the global data struct pointers */
   //Data.Sensors = m_psSensorData;
   //Data.Actuators = m_psActuatorData;
   /* Create lists for blocks, targets, and structures */
   // TODO attached block lists to sensor data


   m_tpLastReset = std::chrono::steady_clock::now();
}

/****************************************/
/****************************************/

void CBeBotController::Destroy() {
   delete m_pcLiftActuatorSystemController;
   delete m_psSensorData;
   delete m_psActuatorData;
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
   
   /* update the tag readings */
   for(auto& t_reading : m_pcTagDetectorAlgorithm->GetReadings()) {
      CRadians pcEulers[3];
      t_reading.Orientation.ToEulerAngles(pcEulers[0], pcEulers[1], pcEulers[2]);

      struct CNearbyLEDsOp : public CPositionalIndex<CLEDEntity>::COperation {
         CNearbyLEDsOp(const CVector3& c_center) :
            m_cCenter(c_center) {}
         virtual bool operator()(CLEDEntity& c_led) {
            if(Distance(c_led.GetPosition(), m_cCenter) < 0.0225) {
               m_vecLEDs.push_back(&c_led);
            }
            return true;
         }
         std::vector<CLEDEntity*> m_vecLEDs;
         CVector3 m_cCenter;
      } cNearbyLEDsOp(t_reading.GlobalPosition);

      /* Use the position of the tag to get the LED colors */
      m_pcLEDIndex->ForEntitiesInBoxRange(t_reading.GlobalPosition,
                                          CVector3(0.05,0.05,0.05),
                                          cNearbyLEDsOp);

      /* Add an empty block to list */
      m_tDetectedBlockList.emplace_back(tpControlStep);
      SBlock& sBlock = m_tDetectedBlockList.back();
      /* Add an empty tag to the block and make a reference to it */
      std::vector<STag>& vecBlockTags = sBlock.Tags;
      vecBlockTags.emplace_back();
      STag& sTag = vecBlockTags.back();

      



      /* TODO:
         1. simulate the lift actuator control loop - done
         2. read sensors
         3. update data structures
         4. step state machine
         5. write back to actuators
      */


      std::cout << t_reading.Payload << ": ";
      for(CLEDEntity* pc_led : cNearbyLEDsOp.m_vecLEDs) {
         std::cout << pc_led->GetColor() << ", ";
      }
      std::cout << std::endl;

     
      std::cout << std::fixed << std::setprecision(3) << t_reading.Payload << " ["
                << t_reading.Position.GetX() << ", "
                << t_reading.Position.GetY() << ", "
                << t_reading.Position.GetZ() << "]"
                << std::endl;

      std::cout << "(" << t_reading.HorizontalIndex << ", " << t_reading.VerticalIndex << ")" << std::endl;
   }
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
