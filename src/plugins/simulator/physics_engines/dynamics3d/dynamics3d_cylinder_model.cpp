/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_cylinder_model.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "dynamics3d_cylinder_model.h"

#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_engine.h>

namespace argos {

   /****************************************/
   /****************************************/

   CDynamics3DCylinderModel::CDynamics3DCylinderModel(CDynamics3DEngine& c_engine,
                                  CCylinderEntity& c_cylinder) :
      CDynamics3DModel(c_engine, c_cylinder.GetEmbodiedEntity()),
      m_cCylinderEntity(c_cylinder) {
      
      /* When defining size of objects we must manually swap the Z and Y components */
      m_pcCylinderCollisionShape = new btCylinderShape(btVector3(c_cylinder.GetRadius(),
                                                                 c_cylinder.GetHeight() * 0.5f,
                                                                 c_cylinder.GetRadius()));
      
      //fprintf(stderr, "[init] position of %s in ARGoS: [%.3f, %.3f, %.3f]\n", m_cCylinderEntity.GetId().c_str(), GetEmbodiedEntity().GetInitPosition().GetX(), GetEmbodiedEntity().GetInitPosition().GetY(),GetEmbodiedEntity().GetInitPosition().GetZ());

      btTransform cModelTransform(ARGoSToBullet(GetEmbodiedEntity().GetInitOrientation()),
                                  ARGoSToBullet(GetEmbodiedEntity().GetInitPosition()));
      
      m_pcCylinderMotionState = new btDefaultMotionState(cModelTransform,
                                                         btTransform(btQuaternion(0.0f, 0.0f, 0.0f, 1.0f),
                                                                     btVector3(0.0f, -c_cylinder.GetHeight() * 0.5f, 0.0f)));
          
      if(c_cylinder.GetEmbodiedEntity().IsMovable()) {
         btVector3 cInertia;
         m_pcCylinderCollisionShape->calculateLocalInertia(c_cylinder.GetMass(), cInertia);
         m_pcCylinderRigidBody = new btRigidBody(btRigidBody::btRigidBodyConstructionInfo(
            c_cylinder.GetMass(), m_pcCylinderMotionState, m_pcCylinderCollisionShape, cInertia));
      }
      else {
         m_pcCylinderRigidBody = new btRigidBody(btRigidBody::btRigidBodyConstructionInfo(
            0.0f, m_pcCylinderMotionState, m_pcCylinderCollisionShape, btVector3(0.0f,0.0f,0.0f)));
      }

      //fprintf(stderr, "[init] position of %s in Bullet: [%.3f, %.3f, %.3f]\n", m_cCylinderEntity.GetId().c_str(), m_pcCylinderRigidBody->getWorldTransform().getOrigin().getX(), m_pcCylinderRigidBody->getWorldTransform().getOrigin().getY(),m_pcCylinderRigidBody->getWorldTransform().getOrigin().getZ() );
      
      m_mapLocalRigidBodies["body"] = m_pcCylinderRigidBody;
   }
   
   /****************************************/
   /****************************************/
   
   CDynamics3DCylinderModel::~CDynamics3DCylinderModel() {
      delete m_pcCylinderRigidBody;
      delete m_pcCylinderMotionState;
      delete m_pcCylinderCollisionShape;
    }
   
   /****************************************/
   /****************************************/

   void CDynamics3DCylinderModel::UpdateEntityStatus() {
      if(m_cCylinderEntity.GetEmbodiedEntity().IsMovable()) {      
         
         //fprintf(stderr, "position of %s in Bullet: [%.3f, %.3f, %.3f]\n", m_cCylinderEntity.GetId().c_str(), m_pcCylinderRigidBody->getWorldTransform().getOrigin().getX(), m_pcCylinderRigidBody->getWorldTransform().getOrigin().getY(),m_pcCylinderRigidBody->getWorldTransform().getOrigin().getZ() );

         //fprintf(stderr, "position of %s in ARGoS (before update): [%.3f, %.3f, %.3f]\n", m_cCylinderEntity.GetId().c_str(), GetEmbodiedEntity().GetPosition().GetX(), GetEmbodiedEntity().GetPosition().GetY(),GetEmbodiedEntity().GetPosition().GetZ());
         
         //const btTransform& cUpdateTransform = m_pcCylinderRigidBody->getWorldTransform();
         const btTransform& cUpdateTransform = m_pcCylinderMotionState->m_graphicsWorldTrans;
         
         GetEmbodiedEntity().SetPosition(BulletToARGoS(cUpdateTransform.getOrigin()));
         GetEmbodiedEntity().SetOrientation(BulletToARGoS(cUpdateTransform.getRotation()));

         //fprintf(stderr, "position of %s in ARGoS: [%.3f, %.3f, %.3f]\n", m_cCylinderEntity.GetId().c_str(), GetEmbodiedEntity().GetPosition().GetX(), GetEmbodiedEntity().GetPosition().GetY(),GetEmbodiedEntity().GetPosition().GetZ());
      
         /* Update components */
         m_cCylinderEntity.UpdateComponents();
      }
   }

   /****************************************/
   /****************************************/

   /*void CDynamics3DCylinderModel::CalculateBoundingBox() {
      btVector3 cAABBMin, cAABBMax;
      btTransform cTransform;
      m_pcCylinderMotionState->getWorldTransform(cTransform);
      m_pcCylinderCollisionShape->getAabb(cTransform, cAABBMin, cAABBMax);
      GetBoundingBox().MinCorner = BulletToARGoS(cAABBMin);
      GetBoundingBox().MaxCorner = BulletToARGoS(cAABBMax);
      }*/

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_DYNAMICS3D_OPERATIONS_ON_ENTITY(CCylinderEntity, CDynamics3DCylinderModel);

}
