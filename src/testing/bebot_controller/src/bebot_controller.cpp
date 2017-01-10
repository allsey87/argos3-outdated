
/**
 * @file <argos3/testing/bebot_controller/src/bebot_controller.cpp>
 *
 * @author Michael Allwright <allsey87@gmail.com>
 */

#include "bebot_controller.h"
#include <argos3/core/utility/logging/argos_log.h>
#include <iomanip>

#include "pyramid_experiment.h"

#define NUM_CHASSIS_DEVICES 12
#define RF_HISTORY_LEN 5

#define RF_FRONT_IDX 12
#define RF_UNDERNEATH_IDX 13
#define RF_LEFT_IDX 14
#define RF_RIGHT_IDX 15
#define EM_DISCHARGE_RATIO 0.975
#define EM_CHARGE_INCR 10
#define DDS_VELOCITY_SCALE 0.35

namespace argos {

   Real ScaleProximityValue(Real f_in) {
      return (5432.4 * std::pow(f_in, 87.70362));
   }

   Real ScaleChassisProximityValue(Real f_in) {
      return (1253.394 - 5493.967*f_in + 7164.404*f_in*f_in);
   }

   /****************************************/
   /****************************************/

   CBeBotController::CBeBotController() :
      m_pcJointsSensor(nullptr),
      m_pcProximitySensor(nullptr),
      m_pcRadiosSensor(nullptr),
      m_pcCamerasSensor(nullptr),
      m_psRadioConfiguration(nullptr),
      m_pcLEDDetectorAlgorithm(nullptr),
      m_pcTagDetectorAlgorithm(nullptr),
      m_pcElectromagnets(nullptr),
      m_pcJointsActuator(nullptr),
      m_pcRadiosActuator(nullptr),
      m_pcFrontLeftWheelJoint(nullptr),
      m_pcFrontRightWheelJoint(nullptr),
      m_pcRearLeftWheelJoint(nullptr),
      m_pcRearRightWheelJoint(nullptr),
      m_pcLiftActuatorSystemController(nullptr),
      m_psSensorData(nullptr),
      m_psActuatorData(nullptr),
      m_pcBlockDetector(nullptr),
      m_pcBlockTracker(nullptr),
      m_pcStructureAnalyser(nullptr),
      m_pcStateMachine(nullptr) {}

   /****************************************/
   /****************************************/

   CBeBotController::~CBeBotController() {
   }

   /****************************************/
   /****************************************/

   void CBeBotController::Init(TConfigurationNode& t_tree) {
      /* sensors */
      m_pcJointsSensor = GetSensor<CCI_PrototypeJointsSensor>("joints");   
      m_pcProximitySensor = GetSensor<CCI_PrototypeProximitySensor>("prototype_proximity");
      m_pcCamerasSensor = GetSensor<CCI_CamerasSensor>("cameras");
      /* camera sensor algorithms */   
      m_pcLEDDetectorAlgorithm = m_pcCamerasSensor->GetAlgorithm<CCI_CamerasSensorLEDDetectorAlgorithm>("duovero_camera","led_detector");
      m_pcTagDetectorAlgorithm = m_pcCamerasSensor->GetAlgorithm<CCI_CamerasSensorTagDetectorAlgorithm>("duovero_camera","tag_detector");
      /* radios */
      m_pcRadiosSensor = GetSensor<CCI_PrototypeRadiosSensor>("radios");
      m_pcRadiosActuator = GetActuator<CCI_PrototypeRadiosActuator>("radios");
      m_psRadioConfiguration = &(m_pcRadiosActuator->GetConfigurations()[0]);
      /* electromagnets */
      m_pcElectromagnets = GetActuator<CCI_PrototypeElectromagnetsActuator>("electromagnets");
      /* actuators */
      m_pcJointsActuator = GetActuator<CCI_PrototypeJointsActuator>("joints");
      /* wheels */   
      m_pcFrontLeftWheelJoint = &(m_pcJointsActuator->GetJointActuator("wheel-front-left:lower-chassis", CCI_PrototypeJointsActuator::ANGULAR_Y));
      m_pcFrontRightWheelJoint = &(m_pcJointsActuator->GetJointActuator("wheel-front-right:lower-chassis", CCI_PrototypeJointsActuator::ANGULAR_Y));
      m_pcRearLeftWheelJoint = &(m_pcJointsActuator->GetJointActuator("wheel-rear-left:lower-chassis", CCI_PrototypeJointsActuator::ANGULAR_Y));
      m_pcRearRightWheelJoint = &(m_pcJointsActuator->GetJointActuator("wheel-rear-right:lower-chassis", CCI_PrototypeJointsActuator::ANGULAR_Y));
      /* issue reset */
      Reset();
   }

   /****************************************/
   /****************************************/

   void CBeBotController::Reset() {
      /* reset the LAS controller */
      delete m_pcLiftActuatorSystemController;
      /* delete modules */
      delete m_pcStructureAnalyser;
      delete m_pcBlockTracker;
      delete m_pcBlockDetector;
      /* delete the state machine */
      delete m_pcStateMachine;
      /* delete data structs */
      delete m_psSensorData;
      delete m_psActuatorData;
      /* recreate data structs */
      m_psSensorData = new CBlockDemo::SSensorData;
      m_psActuatorData = new CBlockDemo::SActuatorData;
      /* recreate the state machine */
      m_pcStateMachine = new CFiniteStateMachine(m_psSensorData, m_psActuatorData);
      /* recreate modules */
      m_pcBlockDetector = new CBlockDetector;
      m_pcBlockDetector->SetCameraMatrix(m_pcCamerasSensor->GetCameraMatrix("duovero_camera"));
      m_pcBlockDetector->SetDistortionParameters(m_pcCamerasSensor->GetDistortionParameters("duovero_camera"));
      m_pcBlockTracker = new CBlockTracker(3u, 0.05f);
      m_pcStructureAnalyser = new CStructureAnalyser;
      
      /* recreate the LAS controller */
      m_pcLiftActuatorSystemController = new CLiftActuatorSystemController(
         m_pcJointsSensor->GetJointSensor("lift-fixture:vertical-link"),
         &(m_pcJointsActuator->GetJointActuator("lift-fixture:vertical-link", CCI_PrototypeJointsActuator::LINEAR_Z))
      );
      /* clear out detection lists */
      m_tDetectedBlockList.clear();
      /* is this required? */
      SetElectromagnetCurrent(0.0);
      SetTargetVelocity(0.0,0.0);
      /* set the experiment start time to epoch */
      m_tpExperimentStart = std::chrono::time_point<std::chrono::steady_clock>();
   }

   /****************************************/
   /****************************************/

   void CBeBotController::Destroy() {
      delete m_pcLiftActuatorSystemController;
      delete m_psSensorData;
      delete m_psActuatorData;
      delete m_pcBlockDetector;
      delete m_pcBlockTracker;
      delete m_pcStructureAnalyser;
   }

   /****************************************/
   /****************************************/

   void CBeBotController::ControlStep() {
      /* update the state machine clock sensor */
      m_psSensorData->Clock.ExperimentStart = m_tpExperimentStart;
      m_psSensorData->Clock.Ticks++;
      m_psSensorData->Clock.Time = m_tpExperimentStart + m_psSensorData->Clock.Ticks * std::chrono::milliseconds(200);
      /* do block, target and structure detection */
      const CCI_CamerasSensorTagDetectorAlgorithm::SReading::TList& tTagReadings =
         m_pcTagDetectorAlgorithm->GetReadings();
      const CCI_CamerasSensorLEDDetectorAlgorithm::SReading::TList& tLedReadings =
         m_pcLEDDetectorAlgorithm->GetReadings();
      /* clear out the old detections */
      m_tDetectedBlockList.clear();
      /* detect blocks */
      m_pcBlockDetector->Detect(tTagReadings, tLedReadings, m_tDetectedBlockList);
      /* associate targets */
      m_pcBlockTracker->AssociateAndTrackTargets(m_psSensorData->Clock.Time,
                                                 m_tDetectedBlockList,
                                                 m_psSensorData->ImageSensor.Detections.Targets);
      /* detect structures */
      m_pcStructureAnalyser->DetectStructures(m_psSensorData->ImageSensor.Detections.Targets,
                                              m_psSensorData->ImageSensor.Detections.Structures);
      /* get readings from the proximity sensors */
      const CCI_PrototypeProximitySensor::SReading::TVector& t_readings = m_pcProximitySensor->GetReadings();
      /* chassis range finders */
      for(UInt32 un_idx = 0; un_idx < NUM_CHASSIS_DEVICES; un_idx++) {
         m_psSensorData->RangeFinders[un_idx].push_front(ScaleChassisProximityValue(t_readings[un_idx].Value));
         if(m_psSensorData->RangeFinders[un_idx].size() > RF_HISTORY_LEN) {
            m_psSensorData->RangeFinders[un_idx].pop_back();
         }
      }

      /* manipulator module range finders */
      m_psSensorData->ManipulatorModule.RangeFinders.EndEffector = 0;
      m_psSensorData->ManipulatorModule.RangeFinders.Left = ScaleProximityValue(t_readings[RF_LEFT_IDX].Value);
      m_psSensorData->ManipulatorModule.RangeFinders.Right = ScaleProximityValue(t_readings[RF_RIGHT_IDX].Value);
      m_psSensorData->ManipulatorModule.RangeFinders.Front = ScaleProximityValue(t_readings[RF_FRONT_IDX].Value);
      m_psSensorData->ManipulatorModule.RangeFinders.Underneath = ScaleProximityValue(t_readings[RF_UNDERNEATH_IDX].Value);
      /* Emulate the manipulator module */
      Real fTargetPos = m_pcLiftActuatorSystemController->GetTargetPosition();
      Real fPos = m_pcLiftActuatorSystemController->GetPosition();
      m_psSensorData->ManipulatorModule.LiftActuator.LimitSwitches.Top = (fPos > 0.1370); // > 137mm (top is at 137.5mm)
      m_psSensorData->ManipulatorModule.LiftActuator.LimitSwitches.Bottom = (fPos < 0.0005); // < 0.5mm (bottom is at 0.0mm)
      m_psSensorData->ManipulatorModule.LiftActuator.State = (std::abs(fTargetPos - fPos) < 0.005) ?
         CBlockDemo::ELiftActuatorSystemState::INACTIVE : CBlockDemo::ELiftActuatorSystemState::ACTIVE_POSITION_CTRL;
      fPos *= 1000; // m => mm
      m_psSensorData->ManipulatorModule.LiftActuator.EndEffector.Position = fPos; 
      /* Simulate the electromagnet charging/discharging process */
      std::list<uint8_t>& lstCharge = m_psSensorData->ManipulatorModule.LiftActuator.Electromagnets.Charge;
      if(lstCharge.empty()) {
         lstCharge.push_back(0);
      }
      if(m_psActuatorData->ManipulatorModule.EndEffector.FieldMode != CBlockDemo::EGripperFieldMode::DISABLED) {
         lstCharge.push_back(std::ceil(lstCharge.back() * EM_DISCHARGE_RATIO));
      }
      else {
         uint8_t unCharge = lstCharge.back();
         if(unCharge < (std::numeric_limits<uint8_t>::max() - EM_CHARGE_INCR)) {
            unCharge += EM_CHARGE_INCR;
         }
         lstCharge.push_back(unCharge);
      }
      if(lstCharge.size() > 3) {
         lstCharge.pop_front();
      }

      /***************************************************/
      /***************************************************/
      
      /* STEP STATE MACHINE */
      m_pcStateMachine->Step();
      /* Output the state if changed */
      std::ostringstream cStateMachineOutput;
      cStateMachineOutput << *m_pcStateMachine;
      if(cStateMachineOutput.str() != m_strLastOutput) {
         m_strLastOutput = cStateMachineOutput.str();
         std::cout << m_strLastOutput << std::endl;
      }
      /* Output the tracking information */
      std::ostringstream cTrackingInfo;
      for(STarget& s_target : m_psSensorData->ImageSensor.Detections.Targets) {
         if(s_target.Id == m_pcStateMachine->GetData<SGlobalData>().TrackedTargetId) {
            cTrackingInfo << ('(' + std::to_string(s_target.Id) + ')');
         }
         else {
            cTrackingInfo << std::to_string(s_target.Id);
         }
         cTrackingInfo << ' ';
      }
      if(cTrackingInfo.str() != m_strLastTrackingInfo) {
         m_strLastTrackingInfo = cTrackingInfo.str();
         std::cerr << (m_strLastTrackingInfo.empty() ? std::string("()") : m_strLastTrackingInfo) << std::endl;
      }
      
      /***************************************************/
      /***************************************************/
     
      /* Update differential drive system */
      if(m_psActuatorData->DifferentialDriveSystem.Left.UpdateReq || 
         m_psActuatorData->DifferentialDriveSystem.Right.UpdateReq) {
         Real fLeftVelocity = m_psActuatorData->DifferentialDriveSystem.Left.Velocity;
         Real fRightVelocity = m_psActuatorData->DifferentialDriveSystem.Right.Velocity;
         fLeftVelocity *= DDS_VELOCITY_SCALE;
         fRightVelocity *= DDS_VELOCITY_SCALE;
         SetTargetVelocity(fLeftVelocity, fRightVelocity);
         m_psActuatorData->DifferentialDriveSystem.Left.UpdateReq = false;
         m_psActuatorData->DifferentialDriveSystem.Right.UpdateReq = false;
      }
      /* Update the lift actuator system */
      if(m_psActuatorData->ManipulatorModule.LiftActuator.Position.UpdateReq) {
         Real fTargetPosition = m_psActuatorData->ManipulatorModule.LiftActuator.Position.Value;
         fTargetPosition *= 0.001; // mm -> m
         m_pcLiftActuatorSystemController->SetTargetPosition(fTargetPosition);
         m_psActuatorData->ManipulatorModule.LiftActuator.Position.UpdateReq = false;
      }
      m_pcLiftActuatorSystemController->ControlStep();
      /* Update radio output */
      m_psRadioConfiguration->TxData.clear();
      if(m_psActuatorData->ManipulatorModule.NFCInterface.UpdateReq) {
         m_psRadioConfiguration->TxData.emplace_back();
         m_psRadioConfiguration->TxData.back() << m_psActuatorData->ManipulatorModule.NFCInterface.OutboundMessage;
         m_psActuatorData->ManipulatorModule.NFCInterface.UpdateReq = false;
      }
      /* Update electromagnet output */
      if(m_psActuatorData->ManipulatorModule.EndEffector.FieldMode == CBlockDemo::EGripperFieldMode::DISABLED) {
         SetElectromagnetCurrent(0);
      }
      else {
         Real fCharge = m_psSensorData->ManipulatorModule.LiftActuator.Electromagnets.Charge.back();
         if(m_psActuatorData->ManipulatorModule.EndEffector.FieldMode == CBlockDemo::EGripperFieldMode::CONSTRUCTIVE) {
            SetElectromagnetCurrent(fCharge);
         }
         else {
            SetElectromagnetCurrent(-fCharge);
         }
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

   /****************************************/
   /****************************************/
}


