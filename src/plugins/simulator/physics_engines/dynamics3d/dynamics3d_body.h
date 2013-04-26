/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_body.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef DYNAMICS3D_BODY_H
#define DYNAMICS3D_BODY_H

namespace argos {
   
   /****************************************/
   /****************************************/

   class CDynamics3DBody {

   public:

      typedef std::pair<std::string, CDynamics3DBody*> TNamedElement;

      class TNamedVector : public std::vector<TNamedElement> {
      public:
         const TNamedVector::const_iterator Find(const std::string& str_id) const {
            TNamedVector::const_iterator it;
            for(it = this->begin(); it != this->end(); ++it) {
               if(it->first == str_id) break;
            }
            return it;
         }

         const CDynamics3DBody* operator[](UInt32 un_index) const {
            return std::vector<TNamedElement>::operator[](un_index).second;
         }

         CDynamics3DBody* operator[](UInt32 un_index) {
            return std::vector<TNamedElement>::operator[](un_index).second;
         }
      };

   public:

      CDynamics3DBody(btCollisionShape* pc_collision_shape = NULL,
                      const btTransform& c_positional_offset = btTransform::getIdentity(),
                      const btTransform& c_geometric_offset = btTransform::getIdentity(),
                      Real f_mass = 0.0f) :
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

      ~CDynamics3DBody() {
         //@todo these checks should not be required...
         if(m_pcRigidBody != NULL) delete m_pcRigidBody; 
         if(m_pcMotionState != NULL) delete m_pcMotionState;
      }

      

      void Reset() {
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



      void SynchronizeMotionState() {
         m_pcRigidBody->setMotionState(m_pcMotionState);
      }

      inline const btTransform& GetMotionStateTransform() const {
         return m_pcMotionState->m_graphicsWorldTrans;
      }

      inline void SetMotionStateTransform(const btTransform & cTransform) {
         m_pcMotionState->m_graphicsWorldTrans = cTransform;
      }

      void ActivateRigidBody() {
         m_pcRigidBody->activate();
      }

      void AddBodyToWorld(btDynamicsWorld * pc_dynamics_world) {
         pc_dynamics_world->addRigidBody(m_pcRigidBody);
      }

      void RemoveBodyFromWorld(btDynamicsWorld * pc_dynamics_world) {
         pc_dynamics_world->removeRigidBody(m_pcRigidBody);
      }


      //@todo Methods to be revised  / updated / deleted

      bool operator==(const btRigidBody* pc_other_body) const {
         return (pc_other_body == m_pcRigidBody);
      }

      inline const btRigidBody& GetRigidBody() const {
         return *m_pcRigidBody;
      }

      inline btRigidBody& GetRigidBody() {
         return *m_pcRigidBody;
      }


      inline btDefaultMotionState& GetMotionState() {
         return *m_pcMotionState;
      }

      inline const btCollisionShape& GetCollisionShape() const {
         return *m_pcCollisionShape;
      }

      // @todo can I use the motion state for this?
      inline const btTransform& GetRigidBodyTransform() const {
         return m_pcRigidBody->getWorldTransform();
      }
    
   public: //@todo make this private add getters setters

      btCollisionShape* m_pcCollisionShape;
      btDefaultMotionState* m_pcMotionState;
      btRigidBody* m_pcRigidBody;

      const btTransform m_cGeometricOffset;
      const btTransform m_cPositionalOffset;
      
      btVector3 m_cInertia;
      Real m_fMass;

   };
}

#endif
