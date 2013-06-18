/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_joint.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef DYNAMICS3D_JOINT_H
#define DYNAMICS3D_JOINT_H

namespace argos {
   class CDynamics3DBody;
}

#include <vector>
#include <string>

#include <argos3/plugins/simulator/physics_engines/dynamics3d/bullet/btBulletDynamicsCommon.h>
#include <argos3/core/utility/datatypes/datatypes.h>

namespace argos {
   
   /****************************************/
   /****************************************/

   class CDynamics3DJoint {

   public:

      typedef std::vector<CDynamics3DJoint*> TVector;

   public:

      struct SJointLimits {
         SJointLimits(const btVector3& c_lower_limit = btVector3(0.0f, 0.0f, 0.0f),
                      const btVector3& c_upper_limit = btVector3(0.0f, 0.0f, 0.0f)) :
            m_cLowerLimit(c_lower_limit),
            m_cUpperLimit(c_upper_limit) {}
         
         btVector3 m_cLowerLimit;
         btVector3 m_cUpperLimit;
      };

      struct SJointActuators {
         struct SActuator {
            SActuator(bool b_enabled = false,
                      Real f_force = 0.0f,
                      Real f_target_velocity = 0.0f) :
               m_bEnabled(b_enabled),
               m_fForce(f_force),
               m_fTargetVelocity(f_target_velocity) {}

            bool m_bEnabled;
            Real m_fForce;
            Real m_fTargetVelocity;
         };
         
         SJointActuators(const SActuator& s_x = SActuator(),
                         const SActuator& s_y = SActuator(),
                         const SActuator& s_z = SActuator()) :
            m_sX(s_x), m_sY(s_y), m_sZ(s_z) {}

         SActuator m_sX, m_sY, m_sZ;
      };

   public:

      enum EAxis {
         LINEAR_X = 0,
         LINEAR_Y = 1,
         LINEAR_Z = 2,
         ANGULAR_X = 3,
         ANGULAR_Y = 4,
         ANGULAR_Z = 5
      };

      const static UInt8 ANGULAR_AXIS_OFFSET = 3;

   public:

      CDynamics3DJoint(const std::string& str_id,
                       CDynamics3DBody& c_body_a,
                       CDynamics3DBody& c_body_b,
                       const btTransform& c_reference_frame_in_a,
                       const btTransform& c_reference_frame_in_b,
                       const SJointLimits& c_linear_limits,
                       const SJointLimits& c_angular_limits,
                       const SJointActuators& c_linear_actuators,
                       const SJointActuators& c_angular_actuators,
                       bool b_use_reference_linear_frame_a = true,
                       bool b_disable_linked_body_collisions = false);
         

      ~CDynamics3DJoint();

      const std::string& GetId() const {
         return m_strId;
      }

      void Reset();

      void SetActuatorParameters(EAxis e_axis,
                                 bool b_enable,
                                 Real f_max_force);

      void SetActuatorTargetVelocity(EAxis e_axis,
                                     Real f_target_velocity);
      
      void AddJointToWorld(btDynamicsWorld* pc_dynamics_world);

      void RemoveJointFromWorld(btDynamicsWorld* pc_dynamics_world);

      btTransform GetSensorReading() {
         return m_pcJoint->getCalculatedTransformA().inverse() * m_pcJoint->getCalculatedTransformB();
      }
    
   private:
      std::string m_strId;

      btGeneric6DofConstraint* m_pcJoint;

      CDynamics3DBody& m_cBodyA;
      CDynamics3DBody& m_cBodyB;

      const btTransform m_cReferenceFrameInA;
      const btTransform m_cReferenceFrameInB;

      const SJointLimits m_sLinearLimits;
      const SJointLimits m_sAngularLimits;
      const SJointActuators m_sLinearActuators;
      const SJointActuators m_sAngularActuators;

      const bool m_bUseLinearReferenceFrameA;
      const bool m_bDisableLinkedBodyCollisions;

   public:

      // Useful joint limits
      const static SJointLimits m_cFreeAxisX;
      const static SJointLimits m_cFreeAxisY;
      const static SJointLimits m_cFreeAxisZ;
      const static SJointLimits m_cFreeAxisXY;
      const static SJointLimits m_cFreeAxisXZ;
      const static SJointLimits m_cFreeAxisYZ;
      const static SJointLimits m_cFreeAxisXYZ;
      const static SJointLimits m_cLockAxes;

   };

   /****************************************/
   /****************************************/

   bool operator==(const CDynamics3DJoint* pc_dyn3d_joint, const std::string& str_id);

   /****************************************/
   /****************************************/
}

#endif
