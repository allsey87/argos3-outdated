/**
 * @file <argos3/plugins/robots/robot/control_interface/ci_robot_rotational_joint_actuator.h>
 *
 * @brief This file provides the common interface definition of the rotational joint actuator.
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef CCI_ROBOT_ROTATIONAL_JOINT_ACTUATOR_H
#define CCI_ROBOT_ROTATIONAL_JOINT_ACTUATOR_H

namespace argos {
   class CCI_RobotRotationalJointActuator;
}

#include <argos3/core/control_interface/ci_actuator.h>
#include <argos3/core/utility/math/angles.h>

namespace argos {

   class CCI_RobotRotationalJointActuator : virtual public CCI_Actuator {

   public:

      /** Rotational joint modes */
      enum ERotationalJointModes {
         MODE_OFF,
         MODE_PASSIVE,
         MODE_SPEED_CONTROL,
         MODE_POSITION_CONTROL,
      };

   public:

      static const CRange<SInt32> SPEED_RANGE;
      static const CRange<Real>   NORMALIZED_SPEED_RANGE;

   public:

      /**
       * Destructor.
       */
      virtual ~CCI_RobotRotationalJointActuator() {}

      virtual void SetRotation(const CRadians& c_angle) = 0;

      virtual void SetRotationSpeed(SInt32 n_speed_pulses) = 0;

      virtual void SetMode(ERotationalJointModes e_mode) = 0;

      /**
       * @brief Sets the rotational joint control mode to active, and sets the target rotation to the given one
       *
       * @param c_angle desired rotational joint rotation
       * @see SetMode
       * @see SetRotation
       *
       */
      inline void SetActiveWithRotation(const CRadians& c_angle) {
         SetPositionControlMode();
         SetRotation(c_angle);
      }

      /**
       * @brief Sets the rotational joint control mode to speed control
       *
       * @see SetMode
       */
      inline void SetSpeedControlMode() {
         SetMode(MODE_SPEED_CONTROL);
      }

      /**
       * @brief Sets the rotational joint control mode to position control
       *
       * @see SetMode
       */
      inline void SetPositionControlMode() {
         SetMode(MODE_POSITION_CONTROL);
      }

      /**
       * @brief Sets the rotational joint control mode to passive
       *
       * @see SetMode
       */
      inline void SetPassiveMode() {
         SetMode(MODE_PASSIVE);
      }
   };
}

#endif
