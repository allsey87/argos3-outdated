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
     
      btTransform cModelTransform(ARGoSToBullet(GetEmbodiedEntity().GetInitOrientation()),
                                   ARGoSToBullet(GetEmbodiedEntity().GetInitPosition()));
      
      m_pcBoxMotionState = new btDefaultMotionState(cModelTransform,
                                                    btTransform(btQuaternion(0.0f, 0.0f, 0.0f, 1.0f),
                                                                btVector3(0.0f, -cBoxHalfSize.GetZ(), 0.0f)));
          
      if(c_box.GetEmbodiedEntity().IsMovable()) {
         btVector3 cInteria;
         m_pcBoxCollisionShape->calculateLocalInertia(c_box.GetMass(), cInteria);
         m_pcBoxRigidBody = new btRigidBody(btRigidBody::btRigidBodyConstructionInfo(
            c_box.GetMass(), m_pcBoxMotionState, m_pcBoxCollisionShape, cInteria));
      }
      else {
         m_pcBoxRigidBody = new btRigidBody(btRigidBody::btRigidBodyConstructionInfo(
            0.0f, m_pcBoxMotionState, m_pcBoxCollisionShape, btVector3(0.0f,0.0f,0.0f)));
      }
      
      m_mapLocalRigidBodies["body"] = m_pcBoxRigidBody;
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
      //if(m_cBoxEntity.GetEmbodiedEntity().IsMovable()) {      
         
         //fprintf(stderr, "position of %s in Bullet: [%.3f, %.3f, %.3f]\n", m_cBoxEntity.GetId().c_str(), m_pcBoxRigidBody->getWorldTransform().getOrigin().getX(), m_pcBoxRigidBody->getWorldTransform().getOrigin().getY(),m_pcBoxRigidBody->getWorldTransform().getOrigin().getZ() );

         const btTransform& cUpdateTransform = m_pcBoxMotionState->m_graphicsWorldTrans;
         
         GetEmbodiedEntity().SetPosition(BulletToARGoS(cUpdateTransform.getOrigin()));
         GetEmbodiedEntity().SetOrientation(BulletToARGoS(cUpdateTransform.getRotation()));

         //fprintf(stderr, "position of %s in ARGoS: [%.3f, %.3f, %.3f]\n", m_cBoxEntity.GetId().c_str(), GetEmbodiedEntity().GetPosition().GetX(), GetEmbodiedEntity().GetPosition().GetY(),GetEmbodiedEntity().GetPosition().GetZ());
      
         /* Update components */
         m_cBoxEntity.UpdateComponents();
     //}
   }

   /****************************************/
   /****************************************/

   /*void CDynamics3DBoxModel::CalculateBoundingBox() {
      btVector3 cAABBMin, cAABBMax;
      btTransform cTransform;
      m_pcBoxMotionState->getWorldTransform(cTransform);
      m_pcBoxCollisionShape->getAabb(cTransform, cAABBMin, cAABBMax);
      GetBoundingBox().MinCorner = BulletToARGoS(cAABBMin);
      GetBoundingBox().MaxCorner = BulletToARGoS(cAABBMax);
    }*/

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_DYNAMICS3D_OPERATIONS_ON_ENTITY(CBoxEntity, CDynamics3DBoxModel);

}
