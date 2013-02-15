/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_box_entity.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "dynamics3d_box_entity.h"
#include "dynamics3d_engine.h"

#include <btBulletDynamicsCommon.h>

namespace argos {

   /****************************************/
   /****************************************/

   CDynamics3DBoxEntity::CDynamics3DBoxEntity(CDynamics3DEngine& c_engine,
                                  CBoxEntity& c_box) :
      CDynamics3DEntity(c_engine, c_box.GetEmbodiedEntity()),
      m_cBoxEntity(c_box) {
      
      const CVector3 cBoxHalfSize = c_box.GetSize() * 0.5f;
      
      /* When defining size of objects we must manually swap the Z and Y components */
      m_pcCollisionShape = new btBoxShape(
         btVector3(cBoxHalfSize.GetX(), cBoxHalfSize.GetZ(), cBoxHalfSize.GetY())
      );
      
      m_pcMotionState = new btDefaultMotionState(btTransform(
         ARGoSToBullet(GetEmbodiedEntity().GetOrientation()),
         ARGoSToBullet(GetEmbodiedEntity().GetPosition())
      ));
          
      if(c_box.GetEmbodiedEntity().IsMovable()) {
         btVector3 cInteria;
         m_pcCollisionShape->calculateLocalInertia(c_box.GetMass(), cInteria);
         m_pcRigidBody = new btRigidBody(btRigidBody::btRigidBodyConstructionInfo(
            c_box.GetMass(), m_pcMotionState, m_pcCollisionShape, cInteria));
      }
      else {
         m_pcRigidBody = new btRigidBody(btRigidBody::btRigidBodyConstructionInfo(
            0.0f, m_pcMotionState, m_pcCollisionShape, btVector3(0.0f,0.0f,0.0f)));
      }
   }
   
   /****************************************/
   /****************************************/
   
   CDynamics3DBoxEntity::~CDynamics3DBoxEntity() {
      delete m_pcRigidBody;
      delete m_pcMotionState;
      delete m_pcCollisionShape;
   }
   
   /****************************************/
   /****************************************/

   bool CDynamics3DBoxEntity::CheckIntersectionWithRay(Real& f_t_on_ray,
                                                    const CRay3& c_ray) const {
      
      btVector3 cRayStart = ARGoSToBullet(c_ray.GetStart());
      btVector3 cRayEnd = ARGoSToBullet(c_ray.GetEnd());
      
      btTransform cRayFromTransform(btTransform::getIdentity());
	   btTransform cRayToTransform(btTransform::getIdentity());

	   cRayFromTransform.setOrigin(cRayStart);
	   cRayToTransform.setOrigin(cRayEnd);
      
      btCollisionWorld::ClosestRayResultCallback cResult(cRayStart, cRayEnd);
      
      btCollisionObject cTempCollisionObject;
      
      btTransform cEntityTransform;
      m_pcMotionState->getWorldTransform(cEntityTransform);
            
      btCollisionWorld::rayTestSingle(cRayFromTransform,
                                      cRayToTransform,
                                      &cTempCollisionObject,
                                      m_pcCollisionShape,
                                      cEntityTransform,
                                      cResult);
      
		if (cResult.hasHit()) {
			btVector3 cHitPoint = cResult.m_hitPointWorld;
			f_t_on_ray = (cHitPoint - cRayStart).length();
         return true;
      }
      else {
         return false;
      }
   }
   
   /****************************************/
   /****************************************/
  
   bool CDynamics3DBoxEntity::MoveTo(const CVector3& c_position,
                                     const CQuaternion& c_orientation,
                                     bool b_check_only) {
      
      /* Create a transform to the new location and orientation */   
      btTransform cEntityTransform(ARGoSToBullet(c_orientation), ARGoSToBullet(c_position));
      
      /* Test if this region defined by the location and collision shape is occupied */
      bool bLocationOccupied = 
         m_cEngine.IsRegionOccupied(cEntityTransform, *m_pcCollisionShape);
         
      if(b_check_only == false && bLocationOccupied == false) {
         m_pcRigidBody->setWorldTransform(cEntityTransform);
         GetEmbodiedEntity().SetPosition(c_position);
         GetEmbodiedEntity().SetOrientation(c_orientation);
      }
      return !bLocationOccupied;
   }
   
   /****************************************/
   /****************************************/

   void CDynamics3DBoxEntity::Reset() {
      if(m_cBoxEntity.GetEmbodiedEntity().IsMovable()) {      
         
         /* Reset box position and orientation */
         m_pcMotionState->setWorldTransform(btTransform(
            ARGoSToBullet(GetEmbodiedEntity().GetInitOrientation()),
            ARGoSToBullet(GetEmbodiedEntity().GetInitPosition())
         ));
         
         m_pcRigidBody->setAngularVelocity(btVector3(0.0f, 0.0f, 0.0f));
         m_pcRigidBody->setLinearVelocity(btVector3(0.0f, 0.0f, 0.0f));
         m_pcRigidBody->clearForces();
      }
   }

   /****************************************/
   /****************************************/

   void CDynamics3DBoxEntity::UpdateEntityStatus() {
      if(m_cBoxEntity.GetEmbodiedEntity().IsMovable()) {      
         
         /* Update box position and orientation */
         btTransform cEntityTransform;
         m_pcMotionState->getWorldTransform(cEntityTransform);
         GetEmbodiedEntity().SetPosition(BulletToARGoS(cEntityTransform.getOrigin()));
         GetEmbodiedEntity().SetOrientation(BulletToARGoS(cEntityTransform.getRotation()));
         
         /* Update components */
         m_cBoxEntity.UpdateComponents();
      }
   }

   /****************************************/
   /****************************************/
   
   REGISTER_STANDARD_DYNAMICS3D_OPERATIONS_ON_ENTITY(CBoxEntity, CDynamics3DBoxEntity);

   /****************************************/
   /****************************************/

}
