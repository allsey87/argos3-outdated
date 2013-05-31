/**
 * @file <argos3/testing/experiment/test_footbot_controller.h>
 *
 * @author Carlo Pinciroli <ilpincy@gmail.com>
 */

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/robot/control_interface/ci_robot_joints_actuator.h>
#include <argos3/core/utility/math/vector2.h>

using namespace argos;

class CRobotJointsController : public CCI_Controller {

public:

   CRobotJointsController();
   virtual ~CRobotJointsController();
   virtual void Init(TConfigurationNode& t_tree);
   virtual void Reset();
   virtual void Destroy();
   virtual void ControlStep();

private:

   CCI_RobotJointsActuator* m_pcJoints;
   CCI_RobotJointsActuator::CJointActuator* m_pcJointActuator;

   UInt32 m_unCount;

};
