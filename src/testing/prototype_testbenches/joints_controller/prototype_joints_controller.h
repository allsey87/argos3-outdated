/**
 * @file <argos3/testing/dyn3d_testbench/prototype_joints_controller.h>
 *
 * @author Michael Allwright allsey87@gmail.com>
 */

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/prototype/control_interface/ci_prototype_joints_actuator.h>
#include <argos3/plugins/robots/prototype/control_interface/ci_prototype_joints_sensor.h>
#include <argos3/core/utility/math/vector2.h>

using namespace argos;

class CPrototypeJointsController : public CCI_Controller {

public:

   CPrototypeJointsController();
   virtual ~CPrototypeJointsController();
   virtual void Init(TConfigurationNode& t_tree);
   virtual void Reset();
   virtual void Destroy();
   virtual void ControlStep();

private:

   CCI_PrototypeJointsActuator* m_pcJoints;
   CCI_PrototypeJointsActuator::CJointActuator* m_pcJointActuator;

   CCI_PrototypeJointsSensor* m_pcJointsSensor;
   CCI_PrototypeJointsSensor::CJointSensor* m_pcJointSensor;

   UInt32 m_unCount;

};
