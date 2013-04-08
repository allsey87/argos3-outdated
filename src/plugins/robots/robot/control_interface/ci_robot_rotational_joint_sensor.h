/**
 * @file <argos3/plugins/robots/robot/control_interface/ci_robot_rotational_joint_sensor.h>
 *
 * @brief This file provides the common interface definition of the rotational joint
 * sensor. The sensor provides a measure of the rotation of the joint.
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef CCI_ROBOT_ROTATIONAL_JOINT_SENSOR_H
#define CCI_ROBOT_ROTATIONAL_JOINT_SENSOR_H

namespace argos {
   class CCI_RobotRotationalJointSensor;
}

#include <argos3/core/control_interface/ci_sensor.h>
#include <argos3/core/utility/math/vector2.h>

namespace argos {

   class CCI_RobotRotationalJointSensor : virtual public CCI_Sensor {

   public:

      static const CRange<CRadians> ANGULAR_RANGE;

   public:

      virtual ~CCI_RobotRotationalJointSensor() {}

      inline const CRadians& GetRotation() const {
         return m_cRotation;
      }

   protected:
      
      CRadians m_cRotation;

      friend class CCI_RobotRotationalJointActuator;

   };

}

#endif
