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
      m_pcBoxCollisionShape = new btBoxShape(
         btVector3(cBoxHalfSize.GetX(), cBoxHalfSize.GetZ(), cBoxHalfSize.GetY())
      );
      
      m_pcBoxMotionState = new btDefaultMotionState(btTransform(
         ARGoSToBullet(GetEmbodiedEntity().GetOrientation()),
         ARGoSToBullet(GetEmbodiedEntity().GetPosition())
      ));
          
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
   
   CDynamics3DBoxEntity::~CDynamics3DBoxEntity() {
      delete m_pcBoxRigidBody;
      delete m_pcBoxMotionState;
      delete m_pcBoxCollisionShape;
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
      m_pcBoxMotionState->getWorldTransform(cEntityTransform);
            
      btCollisionWorld::rayTestSingle(cRayFromTransform,
                                      cRayToTransform,
                                      &cTempCollisionObject,
                                      m_pcBoxCollisionShape,
                                      cEntityTransform,
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
  
   bool CDynamics3DBoxEntity::MoveTo(const CVector3& c_position,
                                     const CQuaternion& c_orientation,
                                     bool b_check_only) {
      
      LOG << "Move to location " << c_position << " requested" << std::endl;
      LOG << "Check only mode: " << (b_check_only?"enabled":"disabled") << std::endl;
      
      /* Create a transform to the new location and orientation */   
      btTransform cEntityTransform(ARGoSToBullet(c_orientation), ARGoSToBullet(c_position));
      
      /* Test if this region defined by the location and collision shape is occupied */
      bool bLocationOccupied = 
         m_cEngine.IsRegionOccupied(cEntityTransform, *m_pcBoxCollisionShape);
         
      LOG << "CDynamics3DEngine::IsRegionOccupied returned: " << (bLocationOccupied?"true":"false") << std::endl;
         
      if(b_check_only == false && bLocationOccupied == false) {
         m_vecLocalRigidBodies[0]->setWorldTransform(cEntityTransform);
         GetEmbodiedEntity().SetPosition(c_position);
         GetEmbodiedEntity().SetOrientation(c_orientation);
      }
      
      LOG << "Final location " << GetEmbodiedEntity().GetPosition() << std::endl;
      return !bLocationOccupied;
   }
   
   /****************************************/
   /****************************************/

   void CDynamics3DBoxEntity::Reset() {
   
      for(std::vector<btRigidBody*>::iterator itBody = m_vecLocalRigidBodies.begin(); 
          itBody !=  m_vecLocalRigidBodies.end();
          itBody++) {   
         (*itBody)->setAngularVelocity(btVector3(0.0f, 0.0f, 0.0f));
         (*itBody)->setLinearVelocity(btVector3(0.0f, 0.0f, 0.0f));
         (*itBody)->clearForces();
         
         (*itBody)->getMotionState()->setWorldTransform(btTransform::getIdentity());
      }
     
      if(m_cBoxEntity.GetEmbodiedEntity().IsMovable()) {      
         
         /* Reset box position and orientation */
         m_pcBoxMotionState->setWorldTransform(btTransform(
            ARGoSToBullet(GetEmbodiedEntity().GetInitOrientation()),
            ARGoSToBullet(GetEmbodiedEntity().GetInitPosition())
         ));
      }
      
      UpdateEntityStatus();
   }

   /****************************************/
   /****************************************/

   void CDynamics3DBoxEntity::UpdateEntityStatus() {
      if(m_cBoxEntity.GetEmbodiedEntity().IsMovable()) {      
         
         /* Update box position and orientation */
         btTransform cEntityTransform;
         m_pcBoxMotionState->getWorldTransform(cEntityTransform);
         GetEmbodiedEntity().SetPosition(BulletToARGoS(cEntityTransform.getOrigin()));
         GetEmbodiedEntity().SetOrientation(BulletToARGoS(cEntityTransform.getRotation()));
         
         /* Update components */
         m_cBoxEntity.UpdateComponents();
      }
   }


   REGISTER_STANDARD_DYNAMICS3D_OPERATIONS_ON_ENTITY(CBoxEntity, CDynamics3DBoxEntity);

   /****************************************/
   /****************************************/

}
