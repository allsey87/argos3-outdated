/**
 * @file <argos3/plugins/robots/robotic-arm/control_interface/ci_robot_rotational_joint_actuator.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "ci_robot_rotational_joint_actuator.h"

namespace argos {

   /****************************************/
   /****************************************/

    const CRange<SInt32> CCI_RobotRotationalJointActuator::SPEED_RANGE(-4,4);
    const CRange<Real> CCI_RobotRotationalJointActuator::NORMALIZED_SPEED_RANGE(-1.0,1.0);

   /****************************************/
   /****************************************/

}
