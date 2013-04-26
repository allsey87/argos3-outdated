/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_joint.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef DYNAMICS3D_JOINT_H
#define DYNAMICS3D_JOINT_H

namespace argos {
   
   /****************************************/
   /****************************************/

   class CDynamics3DJoint {

   public:

      class TVector : public std::vector<CDynamics3DJoint*> {
      public:
         TVector::iterator Find(const std::string& str_id) {
            TVector::iterator it;
            for(it = this->begin(); it != this->end(); ++it) {
               if(it->GetId() == str_id) break;
            }
            return it;
         }

         CDynamics3DJoint* operator[](const std::string& str_id) {
            return *Find(str_id);
         }
      };

   public:

      struct SJointLimits {
         SJointLimits(const btVector3& c_lower_limit = btVector3(0.0f, 0.0f, 0.0f),
                      const btVector3& c_upper_limit = btVector3(0.0f, 0.0f, 0.0f)) :
            m_cLowerLimit(c_lower_limit),
            m_cUpperLimit(c_upper_limit) {}
         
         const btVector3 m_cLowerLimit;
         const btVector3 m_cUpperLimit;
      };

   public:

      struct Actuator {
         enum EAxis {
            LINEAR_X = 0,
            LINEAR_Y = 1,
            LINEAR_Z = 2,
            ANGULAR_X = 3,
            ANGULAR_Y = 4,
            ANGULAR_Z = 5
         };

         const static UInt8 ANGULAR_AXIS_OFFSET = 3;
      };

   public:

      CDynamics3DJoint(const std::string& str_id,
                            CDynamics3DBody& c_body_a,
                            CDynamics3DBody& c_body_b,
                            const btTransform& c_reference_frame_in_a,
                            const btTransform& c_reference_frame_in_b,
                            const SJointLimits& c_linear_limits,
                            const SJointLimits& c_angular_limits,
                            bool b_use_reference_linear_frame_a = true,
                            bool b_disable_linked_body_collisions = false) :
         m_strId(str_id),
         m_cReferenceFrameInA(c_reference_frame_in_a),
         m_cReferenceFrameInB(c_reference_frame_in_b),
         m_sLinearLimits(c_linear_limits),
         m_sAngularLimits(c_angular_limits),
         m_bUseLinearReferenceFrameA(b_use_reference_linear_frame_a),
         m_bDisableLinkedBodyCollisions(b_disable_linked_body_collisions) {

         m_pcJoint = new btGeneric6DofConstraint(m_cBodyA.GetRigidBody(),
                                                 m_cBodyB.GetRigidBody(),
                                                 m_cReferenceFrameInA,
                                                 m_cReferenceFrameInB,
                                                 m_bUseLinearReferenceFrameA);

         m_pcJoint->setLinearLowerLimit(m_sLinearLimits.m_cLowerLimit);
         m_pcJoint->setLinearUpperLimit(m_sLinearLimits.m_cUpperLimit);
         m_pcJoint->setAngularLowerLimit(m_sAngularLimits.m_cLowerLimit);
         m_pcJoint->setAngularUpperLimit(m_sAngularLimits.m_cUpperLimit);
      }  
         

      ~CDynamics3DJoint() {
         delete m_pcJoint;
      }

      const std::string& GetId() {
         return m_strId;
      }

      void Reset() {
         /* delete the joint */
         delete m_pcJoint;

         /* recreate it */
         m_pcJoint = new btGeneric6DofConstraint(m_cBodyA.GetRigidBody(),
                                                 m_cBodyB.GetRigidBody(),
                                                 m_cReferenceFrameInA,
                                                 m_cReferenceFrameInB,
                                                 m_bUseLinearReferenceFrameA);
         /* set the limits */
         m_pcJoint->setLinearLowerLimit(m_sLinearLimits.m_cLowerLimit);
         m_pcJoint->setLinearUpperLimit(m_sLinearLimits.m_cUpperLimit);
         m_pcJoint->setAngularLowerLimit(m_sAngularLimits.m_cLowerLimit);
         m_pcJoint->setAngularUpperLimit(m_sAngularLimits.m_cUpperLimit);

      }

      void SetActuatorParameters(Actuator::EAxis e_axis
                                 bool b_enable,
                                 Real f_max_force) {

         switch(e_axis) {
         case Actuator::LINEAR_X:
         case Actuator::LINEAR_Y:
         case Actuator::LINEAR_Z:
            m_pcJoint->getTranslationLimitMotor->m_enableMotor[e_axis] = b_enable;
            m_pcJoint->getTranslationLimitMotor->m_maxMotorForce[e_axis] = f_max_force;
            break;
         case Actuator::ANGULAR_X:
         case Actuator::ANGULAR_Y:
         case Actuator::ANGULAR_Z:
            m_pcJoint->getRotationalLimitMotor(e_axis - Actuator::ANGULAR_AXIS_OFFSET)->m_enableMotor = b_enable;
            m_pcJoint->getRotationalLimitMotor(e_axis - Actuator::ANGULAR_AXIS_OFFSET)->m_maxMotorForce = f_max_force;
            break;
         }
      }

      void SetActuatorTargetVelocity(Actuator::EAxis e_axis,
                                     Real f_target_velocity) {
         switch(e_axis) {
         case Actuator::LINEAR_X:
         case Actuator::LINEAR_Y:
         case Actuator::LINEAR_Z:
            m_pcJoint->getTranslationLimitMotor->m_targetVelocity[e_axis] = f_target_velocity;
            break;
         case Actuator::ANGULAR_X:
         case Actuator::ANGULAR_Y:
         case Actuator::ANGULAR_Z:
            m_pcJoint->getRotationalLimitMotor(e_axis - Actuator::ANGULAR_AXIS_OFFSET)->m_targetVelocity = f_target_velocity;
            break;
         }
      }
         
      //@todo implement Real GetActuatorVelocity(Actuator::EAxis e_axis);

      //@todo implement Real GetActuatorPosition(Actuator::EAxis e_axis);

      void AddJointToWorld(btDynamicsWorld * pc_dynamics_world) {
         pc_dynamics_world->addConstraint(m_pcJoint, m_bDisableLinkedBodyCollisions);
      }

      void RemoveJointFromWorld(btDynamicsWorld * pc_dynamics_world) {
         pc_dynamics_world->removeContraint(m_pcJoint);
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
}

#endif
