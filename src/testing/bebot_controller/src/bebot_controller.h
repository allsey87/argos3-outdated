/**
 * @file <argos3/testing/bebot_controller/src/bebot_controller.h>
 *
 * @author Michael Allwright <allsey87@gmail.com>
 */

#include <argos3/core/control_interface/ci_controller.h>

#include <argos3/plugins/robots/prototype/control_interface/ci_prototype_joints_actuator.h>
#include <argos3/plugins/robots/prototype/control_interface/ci_prototype_joints_sensor.h>
#include <argos3/plugins/robots/prototype/control_interface/ci_prototype_radios_actuator.h>
#include <argos3/plugins/robots/prototype/control_interface/ci_prototype_radios_sensor.h>
#include <argos3/plugins/robots/prototype/control_interface/ci_prototype_proximity_sensor.h>
#include <argos3/plugins/robots/prototype/control_interface/ci_prototype_electromagnets_actuator.h>
#include <argos3/plugins/robots/prototype/control_interface/ci_cameras_sensor.h>
#include <argos3/plugins/robots/prototype/control_interface/ci_cameras_sensor_algorithms/ci_cameras_sensor_leddetector_algorithm.h>
#include <argos3/plugins/robots/prototype/control_interface/ci_cameras_sensor_algorithms/ci_cameras_sensor_tagdetector_algorithm.h>

#include <array>
#include <list>
#include <cstdint>
#include <chrono>

#include "target.h"
#include "structure.h"

using namespace argos;

enum class EGripperFieldMode : uint8_t {
   CONSTRUCTIVE = 0,
   DESTRUCTIVE = 1,
   DISABLED = 2
};

enum class ELiftActuatorSystemState : uint8_t {
   /* Inactive means the stepper motor is disabled */
   INACTIVE = 0,
   /* Active means the stepper motor is running */
   ACTIVE_POSITION_CTRL = 1,
   ACTIVE_SPEED_CTRL = 2,
   /* Calibration search bottom/top */
   CALIBRATION_SRCH_TOP = 3,
   CALIBRATION_SRCH_BTM = 4,
   /* Not actually a state */
   UNDEFINED = 5,
};

enum class EColor {
   RED, GREEN, BLUE
};

struct SSensorData {
   struct {
      struct {
         STarget::TList Targets;
         SStructure::TList Structures;
      } Detections;
   } ImageSensor;
   struct {
      struct {
         uint16_t EndEffector = 0, Left = 0, Right = 0,
            Front = 0, Underneath = 0;
      } RangeFinders;
      struct {
         struct {
            bool Top = false, Bottom = false;
         } LimitSwitches;
         struct {
            std::list<uint8_t> Charge;
         } Electromagnets;
         struct {
            uint8_t Position = 0;
         } EndEffector;
         ELiftActuatorSystemState State = ELiftActuatorSystemState::UNDEFINED;
      } LiftActuator;
   } ManipulatorModule;
   struct {
      std::chrono::time_point<std::chrono::steady_clock> ExperimentStart;
      std::chrono::time_point<std::chrono::steady_clock> Time;
      unsigned int Ticks = 0;
   } Clock;
   std::array<std::list<uint16_t>, 12> RangeFinders;
};

struct SActuatorData {
   struct {
      struct {
         int16_t Velocity = 0;
         bool UpdateReq = false;
      } Left, Right;
   } DifferentialDriveSystem;
   struct {
      std::array<EColor, 12> Color;
      std::array<bool, 12> UpdateReq = {};
   } LEDDeck;
   struct {
      struct {
         struct {
            int8_t Value = 0;
            bool UpdateReq = false;
         } Velocity;
         struct {
            uint8_t Value = 0;
            bool UpdateReq = false;
         } Position;
      } LiftActuator;
      struct {
         std::string OutboundMessage;
         bool UpdateReq = false;
      } NFCInterface;
      struct {
         EGripperFieldMode FieldMode =
            EGripperFieldMode::DISABLED;
         bool UpdateReq = false;
      } EndEffector;
   } ManipulatorModule;
};

class CBeBotController : public CCI_Controller {
public:
   CBeBotController();
   virtual ~CBeBotController();
   virtual void Init(TConfigurationNode& t_tree);
   virtual void Reset();
   virtual void Destroy();
   virtual void ControlStep();

private:
   class CLiftActuatorSystemController {
   public:
      CLiftActuatorSystemController(CCI_PrototypeJointsSensor::CJointSensor* pc_sensor,
                                    CCI_PrototypeJointsActuator::CJointActuator* pc_actuator) :
         m_pcSensor(pc_sensor),
         m_pcActuator(pc_actuator) {}

      void SetTargetPosition(Real f_target_position) {
         m_fTargetPosition = f_target_position;
      }

      void Reset() {
         m_pcActuator->SetTargetVelocity(0.0);
      }

      void ControlStep() {
         CVector3 cReading;
         m_pcSensor->GetReading(cReading);
         Real fError = (m_fTargetPosition - cReading.GetZ());
         if(std::abs(fError) > 0.001) {
            m_pcActuator->SetTargetVelocity(fError * m_fKp);
         }
         else {
            m_pcActuator->SetTargetVelocity(0.0);
         }      
      }

   private:
      Real m_fTargetPosition = 0.040;
      Real m_fKp = 1.0;
      CCI_PrototypeJointsSensor::CJointSensor* m_pcSensor;
      CCI_PrototypeJointsActuator::CJointActuator* m_pcActuator;
   };

private:
      

   /* private methods */
   void SetTargetVelocity(Real f_left, Real f_right);
   void SetElectromagnetCurrent(Real f_value);
   /* sensors */
   CCI_PrototypeJointsSensor* m_pcJointsSensor;
   CCI_PrototypeProximitySensor* m_pcProximitySensor;
   CCI_PrototypeRadiosSensor* m_pcRadiosSensor;
   CCI_CamerasSensor* m_pcCamerasSensor;
   /* camera sensor algorithms */
   CCI_CamerasSensorLEDDetectorAlgorithm* m_pcLEDDetectorAlgorithm;
   CCI_CamerasSensorTagDetectorAlgorithm* m_pcTagDetectorAlgorithm;
   /* actuators */
   CCI_PrototypeElectromagnetsActuator* m_pcElectromagnets;
   CCI_PrototypeJointsActuator* m_pcJointsActuator;
   CCI_PrototypeRadiosActuator* m_pcRadiosActuator;
   /* wheels */
   CCI_PrototypeJointsActuator::CJointActuator* m_pcFrontLeftWheelJoint;
   CCI_PrototypeJointsActuator::CJointActuator* m_pcFrontRightWheelJoint;
   CCI_PrototypeJointsActuator::CJointActuator* m_pcRearLeftWheelJoint;
   CCI_PrototypeJointsActuator::CJointActuator* m_pcRearRightWheelJoint;
   /* lift actuator controller system*/
   CLiftActuatorSystemController* m_pcLiftActuatorSystemController;

};

