/**
 * @file <argos3/testing/bebot_controller/src/bebot_controller.h>
 *
 * @author Michael Allwright <allsey87@gmail.com>
 */

#ifndef BEBOT_CONTROLLER_H
#define BEBOT_CONTROLLER_H

namespace argos {
   // forward declarations
}

class CFiniteStateMachine;

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

#include "block_demo.h"
#include "block_detector.h"
#include "block_tracker.h"
#include "structure_analyser.h"

namespace argos {
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

         Real GetPosition() {
            CVector3 cReading;
            m_pcSensor->GetReading(cReading);
            return cReading.GetZ();
         }

         Real GetTargetPosition() {
            return m_fTargetPosition;
         }

         void Reset() {
            m_pcActuator->SetTargetVelocity(0.0);
         }

         void ControlStep() {
            Real fError = m_fTargetPosition - GetPosition();
            bool bNegError = (fError < 0.0);
            if(std::abs(fError) > 0.01) {
               m_pcActuator->SetTargetVelocity(bNegError?-0.15:0.15);
            }
            else if(std::abs(fError) > 0.001) {
               m_pcActuator->SetTargetVelocity(bNegError?-0.075:0.075);
            }
            else {
               m_pcActuator->SetTargetVelocity(0.0);
            }      
         }

      private:
         Real m_fTargetPosition = 0.0;
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
      /* radio configuration for NFC */
      CCI_PrototypeRadiosActuator::SConfiguration* m_psRadioConfiguration;
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
      /* Global data struct pointer */
      CBlockDemo::SSensorData* m_psSensorData;
      CBlockDemo::SActuatorData* m_psActuatorData;
      /* Create lists for blocks, targets, and structures */
      SBlock::TList m_tDetectedBlockList;
      CBlockDetector* m_pcBlockDetector;
      CBlockTracker* m_pcBlockTracker;
      CStructureAnalyser* m_pcStructureAnalyser;
      /* time of the last reset of the controller */
      std::chrono::time_point<std::chrono::steady_clock> m_tpExperimentStart;
      /* state machine */
      CFiniteStateMachine* m_pcStateMachine;
      /* debugging output */
      std::string m_strLastOutput;
      std::string m_strLastTrackingInfo;
   };
}

#endif

