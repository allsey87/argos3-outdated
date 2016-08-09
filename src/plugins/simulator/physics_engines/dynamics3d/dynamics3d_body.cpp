/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_body.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "dynamics3d_body.h"
#include <argos3/core/utility/configuration/argos_exception.h>

namespace argos {

   /****************************************/
   /****************************************/

   CDynamics3DBody::CDynamics3DBody(CDynamics3DModel* pc_parent_model,
                                    const std::string& str_id,
                                    btCollisionShape* pc_collision_shape,
                                    const btTransform& c_positional_offset,
                                    const btTransform& c_geometric_offset,
                                    btScalar f_mass) :
      m_pcParentModel(pc_parent_model),
      m_strId(str_id),
      m_pcCollisionShape(pc_collision_shape),
      m_pcMotionState(NULL),
      m_pcRigidBody(NULL),
      m_cGeometricOffset(c_geometric_offset),
      m_cPositionalOffset(c_positional_offset),
      m_cInertia(btVector3(0.0f, 0.0f, 0.0f)),
      m_fMass(f_mass) {      
      /* calculate the inertia */
      if(m_fMass != 0.0f && m_pcCollisionShape != NULL) {
         m_pcCollisionShape->calculateLocalInertia(m_fMass, m_cInertia);
      }
      /* construct the motion state */
      m_pcMotionState = new btDefaultMotionState(m_cPositionalOffset, m_cGeometricOffset);
      /* construct the rigid body */
      m_pcRigidBody = new btRigidBody(btRigidBody::btRigidBodyConstructionInfo(m_fMass,
                                                                               m_pcMotionState,
                                                                               m_pcCollisionShape,
                                                                               m_cInertia));
      /* set the default surface friction */
      m_pcRigidBody->setFriction(0.5f);
      /* For reverse look up */
      m_pcRigidBody->setUserPointer(this);
   }

   /****************************************/
   /****************************************/
   
   CDynamics3DBody::~CDynamics3DBody() {
      delete m_pcRigidBody; 
      delete m_pcMotionState;
   }

   /****************************************/
   /****************************************/
   
   void CDynamics3DBody::Reset() {
      // recreate the motion state
      delete m_pcMotionState;
      m_pcMotionState = new btDefaultMotionState(m_cPositionalOffset, m_cGeometricOffset);      
      // delete the body and recreate it
      delete m_pcRigidBody;
      m_pcRigidBody = new btRigidBody(btRigidBody::btRigidBodyConstructionInfo(m_fMass,
                                                                               m_pcMotionState,
                                                                               m_pcCollisionShape,
                                                                               m_cInertia));
      m_pcRigidBody->setUserPointer(this);
   }

   /****************************************/
   /****************************************/
   
   const btCollisionShape& CDynamics3DBody::GetCollisionShape() const {
      return *m_pcCollisionShape;
   }

   /****************************************/
   /****************************************/
      
   const btTransform& CDynamics3DBody::GetRigidBodyTransform() const {
      return m_pcRigidBody->getWorldTransform();
   }
   
   /****************************************/
   /****************************************/
   
   const btTransform& CDynamics3DBody::GetPositionalOffset() const {
      return m_cPositionalOffset;
   }
  
   /****************************************/
   /****************************************/

   const btTransform& CDynamics3DBody::GetGeometricOffset() const {
      return m_cGeometricOffset;
   }
  
   /****************************************/
   /****************************************/
   
   const btTransform& CDynamics3DBody::GetMotionStateTransform() const {
      return m_pcMotionState->m_graphicsWorldTrans;
   }

   /****************************************/
   /****************************************/

   btScalar CDynamics3DBody::GetMass() const {
      return m_fMass;
   }

   /****************************************/
   /****************************************/
   
   void CDynamics3DBody::SetMotionStateTransform(const btTransform & cTransform) {
      m_pcMotionState->m_graphicsWorldTrans = cTransform;
   }

   /****************************************/
   /****************************************/

   void CDynamics3DBody::SetDamping(btScalar f_linear_damping, btScalar f_angular_damping) {
      m_pcRigidBody->setDamping(f_linear_damping, f_angular_damping);
   }

   /****************************************/
   /****************************************/
   
   void CDynamics3DBody::SynchronizeMotionState() {
      m_pcRigidBody->setMotionState(m_pcMotionState);
   }

   /****************************************/
   /****************************************/
   
   void CDynamics3DBody::ActivateRigidBody() {
      m_pcRigidBody->activate();
   }


   /****************************************/
   /****************************************/

   bool CDynamics3DBody::IsRigidBodyActive() {
      return m_pcRigidBody->isActive();
   }

   /****************************************/
   /****************************************/

   void CDynamics3DBody::ApplyTorque(const btVector3& c_torque) {
      m_pcRigidBody->applyTorque(c_torque);      
   }

   /****************************************/
   /****************************************/

   void CDynamics3DBody::ApplyForce(const btVector3& c_force, const btVector3& c_offset) {
      m_pcRigidBody->applyForce(c_force, c_offset);
   }

   /****************************************/
   /****************************************/

   const btVector3& CDynamics3DBody::GetTotalForce() const {
      return m_pcRigidBody->getTotalForce();
   }

   /****************************************/
   /****************************************/
   
   void CDynamics3DBody::AddBodyToWorld(btDynamicsWorld * pc_dynamics_world) {
      pc_dynamics_world->addRigidBody(m_pcRigidBody);
   }

   /****************************************/
   /****************************************/
   
   void CDynamics3DBody::RemoveBodyFromWorld(btDynamicsWorld * pc_dynamics_world) {
      pc_dynamics_world->removeRigidBody(m_pcRigidBody);
   }

   /****************************************/
   /****************************************/

   bool operator==(const CDynamics3DBody* pc_dyn3d_body, const std::string& str_id) {
      return (pc_dyn3d_body->GetId()) == str_id;
   }

   /****************************************/
   /****************************************/

}
