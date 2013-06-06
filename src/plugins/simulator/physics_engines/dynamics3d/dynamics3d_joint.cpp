/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_joint.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "dynamics3d_joint.h"

#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_body.h>

namespace argos {
      
   /****************************************/
   /****************************************/

   const CDynamics3DJoint::SJointLimits CDynamics3DJoint::m_cFreeAxisX(
      btVector3(1.0f, 0.0f, 0.0f),
      btVector3(-1.0f, 0.0f, 0.0f));

   const CDynamics3DJoint::SJointLimits CDynamics3DJoint::m_cFreeAxisY(
      btVector3(0.0f, 1.0f, 0.0f),
      btVector3(0.0f, -1.0f, 0.0f));

   const CDynamics3DJoint::SJointLimits CDynamics3DJoint::m_cFreeAxisZ(
      btVector3(0.0f, 0.0f, 1.0f),
      btVector3(0.0f, 0.0f, -1.0f));
   

   const CDynamics3DJoint::SJointLimits CDynamics3DJoint::m_cFreeAxisXY(
      btVector3(1.0f, 1.0f, 0.0f),
      btVector3(-1.0f, -1.0f, 0.0f));

   const CDynamics3DJoint::SJointLimits CDynamics3DJoint::m_cFreeAxisXZ(
      btVector3(1.0f, 0.0f, 1.0f),
      btVector3(-1.0f, 0.0f, -1.0f));

   const CDynamics3DJoint::SJointLimits CDynamics3DJoint::m_cFreeAxisYZ(
      btVector3(0.0f, 1.0f, 1.0f),
      btVector3(0.0f, -1.0f, -1.0f));

   
   const CDynamics3DJoint::SJointLimits CDynamics3DJoint::m_cFreeAxisXYZ(
      btVector3(1.0f, 1.0f, 1.0f),
      btVector3(-1.0f, -1.0f, -1.0f));
   

   const CDynamics3DJoint::SJointLimits CDynamics3DJoint::m_cLockAxes(
      btVector3(0.0f, 0.0f, 0.0f),
      btVector3(0.0f, 0.0f, 0.0f));

   /****************************************/
   /****************************************/
   
   CDynamics3DJoint::CDynamics3DJoint(const std::string& str_id,
                                      CDynamics3DBody& c_body_a,
                                      CDynamics3DBody& c_body_b,
                                      const btTransform& c_reference_frame_in_a,
                                      const btTransform& c_reference_frame_in_b,
                                      const SJointLimits& c_linear_limits,
                                      const SJointLimits& c_angular_limits,
                                      const SJointActuators& c_linear_actuators,
                                      const SJointActuators& c_angular_actuators,
                                      bool b_use_reference_linear_frame_a,
                                      bool b_disable_linked_body_collisions) :
      m_strId(str_id),
      m_cBodyA(c_body_a),
      m_cBodyB(c_body_b),
      m_cReferenceFrameInA(c_reference_frame_in_a),
      m_cReferenceFrameInB(c_reference_frame_in_b),
      m_sLinearLimits(c_linear_limits),
      m_sAngularLimits(c_angular_limits),
      m_sLinearActuators(c_linear_actuators),
      m_sAngularActuators(c_angular_actuators),
      m_bUseLinearReferenceFrameA(b_use_reference_linear_frame_a),
      m_bDisableLinkedBodyCollisions(b_disable_linked_body_collisions) {
      
      m_pcJoint = new btGeneric6DofConstraint(*m_cBodyA.m_pcRigidBody,
                                              *m_cBodyB.m_pcRigidBody,
                                              m_cReferenceFrameInA,
                                              m_cReferenceFrameInB,
                                              m_bUseLinearReferenceFrameA);
      
      m_pcJoint->setLinearLowerLimit(m_sLinearLimits.m_cLowerLimit);
      m_pcJoint->setLinearUpperLimit(m_sLinearLimits.m_cUpperLimit);
      m_pcJoint->setAngularLowerLimit(m_sAngularLimits.m_cLowerLimit);
      m_pcJoint->setAngularUpperLimit(m_sAngularLimits.m_cUpperLimit);
      m_pcJoint->getTranslationalLimitMotor()->m_enableMotor[0] = m_sLinearActuators.m_sX.m_bEnabled;
      m_pcJoint->getTranslationalLimitMotor()->m_maxMotorForce[0] = m_sLinearActuators.m_sX.m_fForce;
      m_pcJoint->getTranslationalLimitMotor()->m_targetVelocity[0] = m_sLinearActuators.m_sX.m_fTargetVelocity;
      m_pcJoint->getTranslationalLimitMotor()->m_enableMotor[1] = m_sLinearActuators.m_sY.m_bEnabled;
      m_pcJoint->getTranslationalLimitMotor()->m_maxMotorForce[1] = m_sLinearActuators.m_sY.m_fForce;
      m_pcJoint->getTranslationalLimitMotor()->m_targetVelocity[1] = m_sLinearActuators.m_sY.m_fTargetVelocity;
      m_pcJoint->getTranslationalLimitMotor()->m_enableMotor[2] = m_sLinearActuators.m_sZ.m_bEnabled;
      m_pcJoint->getTranslationalLimitMotor()->m_maxMotorForce[2] = m_sLinearActuators.m_sZ.m_fForce;
      m_pcJoint->getTranslationalLimitMotor()->m_targetVelocity[2] = m_sLinearActuators.m_sZ.m_fTargetVelocity;
      m_pcJoint->getRotationalLimitMotor(0)->m_enableMotor = m_sAngularActuators.m_sX.m_bEnabled;
      m_pcJoint->getRotationalLimitMotor(0)->m_maxMotorForce = m_sAngularActuators.m_sX.m_fForce;
      m_pcJoint->getRotationalLimitMotor(0)->m_targetVelocity =  m_sAngularActuators.m_sX.m_fTargetVelocity;
      m_pcJoint->getRotationalLimitMotor(1)->m_enableMotor = m_sAngularActuators.m_sY.m_bEnabled;
      m_pcJoint->getRotationalLimitMotor(1)->m_maxMotorForce = m_sAngularActuators.m_sY.m_fForce;
      m_pcJoint->getRotationalLimitMotor(1)->m_targetVelocity =  m_sAngularActuators.m_sY.m_fTargetVelocity;
      m_pcJoint->getRotationalLimitMotor(2)->m_enableMotor = m_sAngularActuators.m_sZ.m_bEnabled;
      m_pcJoint->getRotationalLimitMotor(2)->m_maxMotorForce = m_sAngularActuators.m_sZ.m_fForce;
      m_pcJoint->getRotationalLimitMotor(2)->m_targetVelocity =  m_sAngularActuators.m_sZ.m_fTargetVelocity;
   }  

   /****************************************/
   /****************************************/

   CDynamics3DJoint::~CDynamics3DJoint() {
      delete m_pcJoint;
   }
   
   /****************************************/
   /****************************************/

   void CDynamics3DJoint::Reset() {
      /* delete the joint */
      delete m_pcJoint;   
      /* recreate it */
      m_pcJoint = new btGeneric6DofConstraint(*m_cBodyA.m_pcRigidBody,
                                              *m_cBodyB.m_pcRigidBody,
                                              m_cReferenceFrameInA,
                                              m_cReferenceFrameInB,
                                              m_bUseLinearReferenceFrameA);
      /* set the limits */
      m_pcJoint->setLinearLowerLimit(m_sLinearLimits.m_cLowerLimit);
      m_pcJoint->setLinearUpperLimit(m_sLinearLimits.m_cUpperLimit);
      m_pcJoint->setAngularLowerLimit(m_sAngularLimits.m_cLowerLimit);
      m_pcJoint->setAngularUpperLimit(m_sAngularLimits.m_cUpperLimit);
      m_pcJoint->getTranslationalLimitMotor()->m_enableMotor[0] = m_sLinearActuators.m_sX.m_bEnabled;
      m_pcJoint->getTranslationalLimitMotor()->m_maxMotorForce[0] = m_sLinearActuators.m_sX.m_fForce;
      m_pcJoint->getTranslationalLimitMotor()->m_targetVelocity[0] = m_sLinearActuators.m_sX.m_fTargetVelocity;
      m_pcJoint->getTranslationalLimitMotor()->m_enableMotor[1] = m_sLinearActuators.m_sY.m_bEnabled;
      m_pcJoint->getTranslationalLimitMotor()->m_maxMotorForce[1] = m_sLinearActuators.m_sY.m_fForce;
      m_pcJoint->getTranslationalLimitMotor()->m_targetVelocity[1] = m_sLinearActuators.m_sY.m_fTargetVelocity;
      m_pcJoint->getTranslationalLimitMotor()->m_enableMotor[2] = m_sLinearActuators.m_sZ.m_bEnabled;
      m_pcJoint->getTranslationalLimitMotor()->m_maxMotorForce[2] = m_sLinearActuators.m_sZ.m_fForce;
      m_pcJoint->getTranslationalLimitMotor()->m_targetVelocity[2] = m_sLinearActuators.m_sZ.m_fTargetVelocity;
      m_pcJoint->getRotationalLimitMotor(0)->m_enableMotor = m_sAngularActuators.m_sX.m_bEnabled;
      m_pcJoint->getRotationalLimitMotor(0)->m_maxMotorForce = m_sAngularActuators.m_sX.m_fForce;
      m_pcJoint->getRotationalLimitMotor(0)->m_targetVelocity =  m_sAngularActuators.m_sX.m_fTargetVelocity;
      m_pcJoint->getRotationalLimitMotor(1)->m_enableMotor = m_sAngularActuators.m_sY.m_bEnabled;
      m_pcJoint->getRotationalLimitMotor(1)->m_maxMotorForce = m_sAngularActuators.m_sY.m_fForce;
      m_pcJoint->getRotationalLimitMotor(1)->m_targetVelocity =  m_sAngularActuators.m_sY.m_fTargetVelocity;
      m_pcJoint->getRotationalLimitMotor(2)->m_enableMotor = m_sAngularActuators.m_sZ.m_bEnabled;
      m_pcJoint->getRotationalLimitMotor(2)->m_maxMotorForce = m_sAngularActuators.m_sZ.m_fForce;
      m_pcJoint->getRotationalLimitMotor(2)->m_targetVelocity =  m_sAngularActuators.m_sZ.m_fTargetVelocity;      
   }
   
   /****************************************/
   /****************************************/

   void CDynamics3DJoint::SetActuatorParameters(EAxis e_axis,
                                                bool b_enable,
                                                Real f_max_force) {
      
      switch(e_axis) {
      case LINEAR_X:
      case LINEAR_Y:
      case LINEAR_Z:
         m_pcJoint->getTranslationalLimitMotor()->m_enableMotor[e_axis] = b_enable;
         m_pcJoint->getTranslationalLimitMotor()->m_maxMotorForce[e_axis] = f_max_force;
         break;
      case ANGULAR_X:
      case ANGULAR_Y:
      case ANGULAR_Z:
         m_pcJoint->getRotationalLimitMotor(e_axis - ANGULAR_AXIS_OFFSET)->m_enableMotor = b_enable;
         m_pcJoint->getRotationalLimitMotor(e_axis - ANGULAR_AXIS_OFFSET)->m_maxMotorForce = f_max_force;
         break;
      }
   }

   /****************************************/
   /****************************************/

   void CDynamics3DJoint::SetActuatorTargetVelocity(EAxis e_axis,
                                                    Real f_target_velocity) {
      switch(e_axis) {
      case LINEAR_X:
      case LINEAR_Y:
      case LINEAR_Z:
         m_pcJoint->getTranslationalLimitMotor()->m_targetVelocity[e_axis] = f_target_velocity;
         break;
      case ANGULAR_X:
      case ANGULAR_Y:
      case ANGULAR_Z:
         m_pcJoint->getRotationalLimitMotor(e_axis - ANGULAR_AXIS_OFFSET)->m_targetVelocity = f_target_velocity;
         break;
      }

      /*
      if(e_axis == ANGULAR_X) {
      const btTransform* toFprintf = &m_pcJoint->getCalculatedTransformA();
      fprintf(stderr, "%s: getCalculatedTransformA() : position = [%.3f, %.3f, %.3f], orientation-axis = [%.3f, %.3f, %.3f], orientation-angle = [%.3f]\n", m_strId.c_str(), toFprintf->getOrigin().getX(), toFprintf->getOrigin().getY(), toFprintf->getOrigin().getZ(), toFprintf->getRotation().getAxis().getX(), toFprintf->getRotation().getAxis().getY(), toFprintf->getRotation().getAxis().getZ(), toFprintf->getRotation().getAngle() * 57.2957795131);

      toFprintf = &m_pcJoint->getCalculatedTransformB();
      fprintf(stderr, "%s: getCalculatedTransformB() : position = [%.3f, %.3f, %.3f], orientation-axis = [%.3f, %.3f, %.3f], orientation-angle = [%.3f]\n", m_strId.c_str(), toFprintf->getOrigin().getX(), toFprintf->getOrigin().getY(), toFprintf->getOrigin().getZ(), toFprintf->getRotation().getAxis().getX(), toFprintf->getRotation().getAxis().getY(), toFprintf->getRotation().getAxis().getZ(), toFprintf->getRotation().getAngle() * 57.2957795131);

      toFprintf = &m_pcJoint->getFrameOffsetA();
      fprintf(stderr, "%s: getFrameOffsetA() : position = [%.3f, %.3f, %.3f], orientation-axis = [%.3f, %.3f, %.3f], orientation-angle = [%.3f]\n", m_strId.c_str(), toFprintf->getOrigin().getX(), toFprintf->getOrigin().getY(), toFprintf->getOrigin().getZ(), toFprintf->getRotation().getAxis().getX(), toFprintf->getRotation().getAxis().getY(), toFprintf->getRotation().getAxis().getZ(), toFprintf->getRotation().getAngle() * 57.2957795131);

      toFprintf = &m_pcJoint->getFrameOffsetB();
      fprintf(stderr, "%s: getFrameOffsetB() : position = [%.3f, %.3f, %.3f], orientation-axis = [%.3f, %.3f, %.3f], orientation-angle = [%.3f]\n", m_strId.c_str(), toFprintf->getOrigin().getX(), toFprintf->getOrigin().getY(), toFprintf->getOrigin().getZ(), toFprintf->getRotation().getAxis().getX(), toFprintf->getRotation().getAxis().getY(), toFprintf->getRotation().getAxis().getZ(), toFprintf->getRotation().getAngle() * 57.2957795131);

      toFprintf = new btTransform();
      fprintf(stderr, "%s: getCalculatedTransformA() * getCalculatedTransformB().inverse : position = [%.3f, %.3f, %.3f], orientation-axis = [%.3f, %.3f, %.3f], orientation-angle = [%.3f]\n", m_strId.c_str(), toFprintf->getOrigin().getX(), toFprintf->getOrigin().getY(), toFprintf->getOrigin().getZ(), toFprintf->getRotation().getAxis().getX(), toFprintf->getRotation().getAxis().getY(), toFprintf->getRotation().getAxis().getZ(), toFprintf->getRotation().getAngle() * 57.2957795131);

      }*/
   }

   /****************************************/
   /****************************************/
   
   void CDynamics3DJoint::AddJointToWorld(btDynamicsWorld* pc_dynamics_world) {
      pc_dynamics_world->addConstraint(m_pcJoint, m_bDisableLinkedBodyCollisions);
   }

   /****************************************/
   /****************************************/
   
   void CDynamics3DJoint::RemoveJointFromWorld(btDynamicsWorld* pc_dynamics_world) {
      pc_dynamics_world->removeConstraint(m_pcJoint);
   }

   /****************************************/
   /****************************************/

   bool operator==(const CDynamics3DJoint* pc_dyn3d_joint, const std::string& str_id) {
      return (pc_dyn3d_joint->GetId()) == str_id;
   }

   /****************************************/
   /****************************************/
      
}
