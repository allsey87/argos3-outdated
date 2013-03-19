/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_cylinder_model.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "dynamics3d_cylinder_model.h"
#include "dynamics3d_engine.h"

#include <btBulletDynamicsCommon.h>

namespace argos {

   /****************************************/
   /****************************************/

   CDynamics3DCylinderModel::CDynamics3DCylinderModel(CDynamics3DEngine& c_engine,
                                  CCylinderEntity& c_cylinder) :
      CDynamics3DModel(c_engine, c_cylinder.GetEmbodiedEntity()),
      m_cCylinderEntity(c_cylinder) {
      
      /* When defining size of objects we must manually swap the Z and Y components */
      m_pcCylinderBaseShape = new btCylinderShape(btVector3(c_cylinder.GetRadius(),
                                                            c_cylinder.GetHeight() * 0.5f,
                                                            c_cylinder.GetRadius()));

      m_pcCylinderCollisionShape = new btCompoundShape();

      m_pcCylinderCollisionShape->addChildShape(btTransform(btQuaternion(0,0,0,1),
                                                            btVector3(0.0f, c_cylinder.GetHeight() * 0.5f, 0.0f)),
                                                m_pcCylinderBaseShape);
      
      btTransform cModelTransform(ARGoSToBullet(GetEmbodiedEntity().GetOrientation()),
                                   ARGoSToBullet(GetEmbodiedEntity().GetPosition()));
      
      m_pcCylinderMotionState = new btDefaultMotionState(cModelTransform);
          
      if(c_cylinder.GetEmbodiedEntity().IsMovable()) {
         btVector3 cInteria;
         m_pcCylinderCollisionShape->calculateLocalInertia(c_cylinder.GetMass(), cInteria);
         m_pcCylinderRigidBody = new btRigidBody(btRigidBody::btRigidBodyConstructionInfo(
            c_cylinder.GetMass(), m_pcCylinderMotionState, m_pcCylinderCollisionShape, cInteria));
      }
      else {
         m_pcCylinderRigidBody = new btRigidBody(btRigidBody::btRigidBodyConstructionInfo(
            0.0f, m_pcCylinderMotionState, m_pcCylinderCollisionShape, btVector3(0.0f,0.0f,0.0f)));
      }
      
      m_vecLocalRigidBodies.push_back(m_pcCylinderRigidBody);
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

   bool CDynamics3DCylinderModel::CheckIntersectionWithRay(Real& f_t_on_ray,
                                                    const CRay3& c_ray) const {
      return false;
     
   }
   
   /****************************************/
   /****************************************/
  
   bool CDynamics3DCylinderModel::MoveTo(const CVector3& c_position,
                                     const CQuaternion& c_orientation,
                                     bool b_check_only) {
      
      return false;
   }
   
   /****************************************/
   /****************************************/

   void CDynamics3DCylinderModel::Reset() {
      
      if(m_cCylinderEntity.GetEmbodiedEntity().IsMovable()) {
         btTransform cResetTransform(
            ARGoSToBullet(GetEmbodiedEntity().GetInitOrientation()),
            ARGoSToBullet(GetEmbodiedEntity().GetInitPosition())
         );
         
         m_pcCylinderRigidBody->setWorldTransform(cResetTransform);
      }
     
      for(std::vector<btRigidBody*>::iterator itBody = m_vecLocalRigidBodies.begin(); 
          itBody !=  m_vecLocalRigidBodies.end();
          itBody++) {   
         (*itBody)->setAngularVelocity(btVector3(0.0f, 0.0f, 0.0f));
         (*itBody)->setLinearVelocity(btVector3(0.0f, 0.0f, 0.0f));
         (*itBody)->clearForces();
      }
   }

   /****************************************/
   /****************************************/

   void CDynamics3DCylinderModel::UpdateEntityStatus() {
      if(m_cCylinderEntity.GetEmbodiedEntity().IsMovable()) {      
         
         //fprintf(stderr, "position of %s in Bullet: [%.3f, %.3f, %.3f]\n", m_cCylinderEntity.GetId().c_str(), m_pcCylinderRigidBody->getWorldTransform().getOrigin().getX(), m_pcCylinderRigidBody->getWorldTransform().getOrigin().getY(),m_pcCylinderRigidBody->getWorldTransform().getOrigin().getZ() );

         const btTransform& cUpdateTransform = m_pcCylinderRigidBody->getWorldTransform();
         
         GetEmbodiedEntity().SetPosition(BulletToARGoS(cUpdateTransform.getOrigin()));
         GetEmbodiedEntity().SetOrientation(BulletToARGoS(cUpdateTransform.getRotation()));

         //fprintf(stderr, "position of %s in ARGoS: [%.3f, %.3f, %.3f]\n", m_cCylinderEntity.GetId().c_str(), GetEmbodiedEntity().GetPosition().GetX(), GetEmbodiedEntity().GetPosition().GetY(),GetEmbodiedEntity().GetPosition().GetZ());
      
         /* Update components */
         m_cCylinderEntity.UpdateComponents();
      }
   }

   /****************************************/
   /****************************************/

   void CDynamics3DCylinderModel::CalculateBoundingBox() {
      btVector3 cAABBMin, cAABBMax;
      btTransform cTransform;
      m_pcCylinderMotionState->getWorldTransform(cTransform);
      m_pcCylinderCollisionShape->getAabb(cTransform, cAABBMin, cAABBMax);
      GetBoundingBox().MinCorner = BulletToARGoS(cAABBMin);
      GetBoundingBox().MaxCorner = BulletToARGoS(cAABBMax);
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_DYNAMICS3D_OPERATIONS_ON_ENTITY(CCylinderEntity, CDynamics3DCylinderModel);

}
