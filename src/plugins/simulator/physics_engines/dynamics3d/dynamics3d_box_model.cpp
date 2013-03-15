/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_box_model.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "dynamics3d_box_model.h"
#include "dynamics3d_engine.h"

#include <btBulletDynamicsCommon.h>

namespace argos {

   /****************************************/
   /****************************************/

   CDynamics3DBoxModel::CDynamics3DBoxModel(CDynamics3DEngine& c_engine,
                                  CBoxEntity& c_box) :
      CDynamics3DModel(c_engine, c_box.GetEmbodiedEntity()),
      m_cBoxEntity(c_box) {
      
      const CVector3 cBoxHalfSize = c_box.GetSize() * 0.5f;
      
      /* When defining size of objects we must manually swap the Z and Y components */
      m_pcBoxCollisionShape = new btBoxShape(
         btVector3(cBoxHalfSize.GetX(), cBoxHalfSize.GetZ(), cBoxHalfSize.GetY())
      );
      
      m_pcBoxTransform = new btTransform(
         btQuaternion(0.0, 0.0f, 0.0f, 1.0f),
         btVector3(0.0f, cBoxHalfSize.GetZ(), 0.0f));
      
      btTransform cModelTransform(ARGoSToBullet(GetEmbodiedEntity().GetOrientation()),
                                   ARGoSToBullet(GetEmbodiedEntity().GetPosition()));
      
      m_pcBoxMotionState = new btDefaultMotionState(cModelTransform * (*m_pcBoxTransform));
          
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
      
      m_vecLocalRigidBodies.push_back(m_pcBoxRigidBody);
   }
   
   /****************************************/
   /****************************************/
   
   CDynamics3DBoxModel::~CDynamics3DBoxModel() {
      delete m_pcBoxRigidBody;
      delete m_pcBoxMotionState;
      delete m_pcBoxCollisionShape;
      delete m_pcBoxTransform;
   }
   
   /****************************************/
   /****************************************/

   bool CDynamics3DBoxModel::CheckIntersectionWithRay(Real& f_t_on_ray,
                                                    const CRay3& c_ray) const {
      
      btVector3 cRayStart = ARGoSToBullet(c_ray.GetStart());
      btVector3 cRayEnd = ARGoSToBullet(c_ray.GetEnd());
      
      btTransform cRayFromTransform(btTransform::getIdentity());
	   btTransform cRayToTransform(btTransform::getIdentity());

	   cRayFromTransform.setOrigin(cRayStart);
	   cRayToTransform.setOrigin(cRayEnd);
      
      btCollisionWorld::ClosestRayResultCallback cResult(cRayStart, cRayEnd);
      
      btCollisionObject cTempCollisionObject;
      
      btTransform cModelTransform;
      m_pcBoxMotionState->getWorldTransform(cModelTransform);
            
      btCollisionWorld::rayTestSingle(cRayFromTransform,
                                      cRayToTransform,
                                      &cTempCollisionObject,
                                      m_pcBoxCollisionShape,
                                      cModelTransform,
                                      cResult);
      
		if (cResult.hasHit()) {
			btVector3 cHitPoint = cResult.m_hitPointWorld;
			f_t_on_ray = (cHitPoint - cRayStart).length() / (cRayEnd - cRayStart).length();
         return true;
      }
      else {
         return false;
      }
   }
   
   /****************************************/
   /****************************************/
  
   bool CDynamics3DBoxModel::MoveTo(const CVector3& c_position,
                                     const CQuaternion& c_orientation,
                                     bool b_check_only) {
      
      LOG << "Move to location " << c_position << " requested" << std::endl;
      LOG << "Check only mode: " << (b_check_only?"enabled":"disabled") << std::endl;
      
      /* Create a transform to the new location and orientation */   
      btTransform cModelTransform(ARGoSToBullet(c_orientation), ARGoSToBullet(c_position));
      
      /* Test if this region defined by the location and collision shape is occupied */
      bool bLocationOccupied = 
         m_cEngine.IsRegionOccupied(cModelTransform, *m_pcBoxCollisionShape);
         
      LOG << "CDynamics3DEngine::IsRegionOccupied returned: " << (bLocationOccupied?"true":"false") << std::endl;
         
      if(b_check_only == false && bLocationOccupied == false) {
         m_vecLocalRigidBodies[0]->setWorldTransform(cModelTransform);
         GetEmbodiedEntity().SetPosition(c_position);
         GetEmbodiedEntity().SetOrientation(c_orientation);
      }
      
      LOG << "Final location " << GetEmbodiedEntity().GetPosition() << std::endl;
      return !bLocationOccupied;
   }
   
   /****************************************/
   /****************************************/

   void CDynamics3DBoxModel::Reset() {

      if(m_cBoxEntity.GetEmbodiedEntity().IsMovable()) {      
         
         btTransform cResetTransform(
            ARGoSToBullet(GetEmbodiedEntity().GetInitOrientation()),
            ARGoSToBullet(GetEmbodiedEntity().GetInitPosition())
         );
         
         m_pcBoxRigidBody->getMotionState()->setWorldTransform(cResetTransform * (*m_pcBoxTransform));
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

   void CDynamics3DBoxModel::UpdateEntityStatus() {
      if(m_cBoxEntity.GetEmbodiedEntity().IsMovable()) {      
         
         btTransform cModelTransform;
         
         m_pcBoxRigidBody->getMotionState()->getWorldTransform(cModelTransform);
         
         cModelTransform = m_pcBoxTransform->inverse() * cModelTransform;
         
         GetEmbodiedEntity().SetPosition(BulletToARGoS(cModelTransform.getOrigin()));
         GetEmbodiedEntity().SetOrientation(BulletToARGoS(cModelTransform.getRotation()));
      
         /* Update components */
         m_cBoxEntity.UpdateComponents();
      }
   }

   /****************************************/
   /****************************************/

   void CDynamics3DBoxModel::CalculateBoundingBox() {
      btVector3 cAABBMin, cAABBMax;
      btTransform cTransform;
      m_pcBoxMotionState->getWorldTransform(cTransform);
      m_pcBoxCollisionShape->getAabb(cTransform, cAABBMin, cAABBMax);
      GetBoundingBox().MinCorner = BulletToARGoS(cAABBMin);
      GetBoundingBox().MaxCorner = BulletToARGoS(cAABBMax);
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_DYNAMICS3D_OPERATIONS_ON_ENTITY(CBoxEntity, CDynamics3DBoxModel);

}
