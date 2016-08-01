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

using namespace argos;

class CBeBotController : public CCI_Controller {
public:
   CBeBotController();
   virtual ~CBeBotController();
   virtual void Init(TConfigurationNode& t_tree);
   virtual void Reset();
   virtual void Destroy();
   virtual void ControlStep();

private:
   /* private methods */
   void SetTargetVelocity(double f_left, double f_right);
   void SetElectromagnetCurrent(double f_value);
   /* sensors */
   CCI_PrototypeJointsSensor* m_pcJointsSensor;
   CCI_PrototypeProximitySensor* m_pcProximitySensor;
   CCI_PrototypeRadiosSensor* m_pcRadiosSensor;
   /* actuators */
   CCI_PrototypeElectromagnetsActuator* m_pcElectromagnets;
   CCI_PrototypeJointsActuator* m_pcJointsActuator;
   CCI_PrototypeRadiosActuator* m_pcRadiosActuator;
   /* joints */
   CCI_PrototypeJointsActuator::CJointActuator* m_pcLiftActuatorJoint;
   CCI_PrototypeJointsActuator::CJointActuator* m_pcFrontLeftWheelJoint;
   CCI_PrototypeJointsActuator::CJointActuator* m_pcFrontRightWheelJoint;
   CCI_PrototypeJointsActuator::CJointActuator* m_pcRearLeftWheelJoint;
   CCI_PrototypeJointsActuator::CJointActuator* m_pcRearRightWheelJoint;

};

