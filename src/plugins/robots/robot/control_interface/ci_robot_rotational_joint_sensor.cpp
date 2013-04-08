/**
 * @file <argos3/plugins/robots/robotic-arm/control_interface/ci_robot_rotation_joint_sensor.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "ci_robot_rotational_joint_sensor.h"

namespace argos {
   
   /****************************************/
    /****************************************/
   
   const CRange<CRadians> CCI_RobotRotationalJointSensor::ANGULAR_RANGE(CRadians(-ARGOS_PI), CRadians(ARGOS_PI));
   
   /****************************************/
   /****************************************/
   
}
