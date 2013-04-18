/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_box_model.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "dynamics3d_box_model.h"

#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_engine.h>

namespace argos {

   /****************************************/
   /****************************************/

   CDynamics3DBoxModel::CDynamics3DBoxModel(CDynamics3DEngine& c_engine,
                                  CBoxEntity& c_box) :
      CDynamics3DModel(c_engine, c_box.GetEmbodiedEntity()),
      m_cBoxEntity(c_box) {
      
      const CVector3 cBoxHalfSize = c_box.GetSize() * 0.5f;
      
      /* When defining size of objects we must manually swap the Z and Y components */
      m_pcBoxCollisionShape = new btBoxShape(btVector3(cBoxHalfSize.GetX(),
                                                       cBoxHalfSize.GetZ(), 
                                                       cBoxHalfSize.GetY()));
      
      m_pcBoxMotionState = new btDefaultMotionState(btTransform::getIdentity(),
                                                    btTransform(btQuaternion(0.0f, 0.0f, 0.0f, 1.0f),
                                                                btVector3(0.0f, -cBoxHalfSize.GetZ(), 0.0f)));
            
      btVector3 cInteria(0.0f, 0.0f, 0.0f);
      Real fMass = 0.0f;
 
      if(c_box.GetEmbodiedEntity().IsMovable()) {
         fMass = c_box.GetMass();
         m_pcBoxCollisionShape->calculateLocalInertia(fMass, cInteria);
      }

      m_pcBoxRigidBody = new btRigidBody(btRigidBody::btRigidBodyConstructionInfo(
         fMass, m_pcBoxMotionState, m_pcBoxCollisionShape, cInteria));

      m_mapLocalBodyConfigurations["box"] = SBodyConfiguration("box",
                                                               m_pcBoxCollisionShape,
                                                               m_pcBoxMotionState,
                                                               m_pcBoxRigidBody,
                                                               btTransform::getIdentity(),
                                                               cInteria,
                                                               fMass);
      /* move the model to the specified coordinates */
      SetModelCoordinates(btTransform(ARGoSToBullet(GetEmbodiedEntity().GetInitOrientation()),
                                      ARGoSToBullet(GetEmbodiedEntity().GetInitPosition())));
   }
   
   /****************************************/
   /****************************************/
   
   CDynamics3DBoxModel::~CDynamics3DBoxModel() {
      delete m_pcBoxRigidBody;
      delete m_pcBoxMotionState;
      delete m_pcBoxCollisionShape;
   }
   
   /****************************************/
   /****************************************/

   void CDynamics3DBoxModel::UpdateEntityStatus() {
      if(m_cBoxEntity.GetEmbodiedEntity().IsMovable()) {      
         const btTransform& cUpdateTransform = GetModelCoordinates();
         
         GetEmbodiedEntity().SetPosition(BulletToARGoS(cUpdateTransform.getOrigin()));
         GetEmbodiedEntity().SetOrientation(BulletToARGoS(cUpdateTransform.getRotation()));

         /* Update components */
         m_cBoxEntity.UpdateComponents();
     }
   }

   /****************************************/
   /****************************************/

   btTransform CDynamics3DBoxModel::GetModelCoordinates() const {
      return m_pcBoxMotionState->m_graphicsWorldTrans;
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_DYNAMICS3D_OPERATIONS_ON_ENTITY(CBoxEntity, CDynamics3DBoxModel);

}
