/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_body.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "dynamics3d_body.h"

namespace argos {

   /****************************************/
   /****************************************/
   
   CDynamics3DBody::TVector::iterator CDynamics3DBody::TVector::Find(const std::string& str_id) {
      iterator it;
      for(it = this->begin(); it != this->end(); ++it) {
         if((*it)->GetId() == str_id) break;
      }
      return it;
   }

   CDynamics3DBody::TVector::const_iterator CDynamics3DBody::TVector::Find(const std::string& str_id) const {
      const_iterator it;
      for(it = this->begin(); it != this->end(); ++it) {
         if((*it)->GetId() == str_id) break;
      }
      return it;
   }

   /****************************************/
   /****************************************/
   
   CDynamics3DBody* CDynamics3DBody::TVector::operator[](const std::string& str_id) {
      return *Find(str_id);
   }

   const CDynamics3DBody* CDynamics3DBody::TVector::operator[](const std::string& str_id) const {
      return *Find(str_id);
   }

   CDynamics3DBody* CDynamics3DBody::TVector::operator[](UInt32 un_idx) {
      return std::vector<CDynamics3DBody*>::operator[](un_idx);
   }

   const CDynamics3DBody* CDynamics3DBody::TVector::operator[](UInt32 un_idx) const {
      return std::vector<CDynamics3DBody*>::operator[](un_idx);
   }

   /****************************************/
   /****************************************/

   CDynamics3DBody::CDynamics3DBody(const std::string& str_id,
                                    btCollisionShape* pc_collision_shape,
                                    const btTransform& c_positional_offset,
                                    const btTransform& c_geometric_offset,
                                    Real f_mass) :
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
   }

   /****************************************/
   /****************************************/
   
   const btCollisionShape& CDynamics3DBody::GetCollisionShape() const {
      return *m_pcCollisionShape;
   }

   /****************************************/
   /****************************************/
   
   bool CDynamics3DBody::operator==(const btCollisionObject* pc_collision_object) const {
      return (pc_collision_object == m_pcRigidBody);
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
   
   const btTransform& CDynamics3DBody::GetMotionStateTransform() const {
      return m_pcMotionState->m_graphicsWorldTrans;
   }

   /****************************************/
   /****************************************/
   
   void CDynamics3DBody::SetMotionStateTransform(const btTransform & cTransform) {
      m_pcMotionState->m_graphicsWorldTrans = cTransform;
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
}
