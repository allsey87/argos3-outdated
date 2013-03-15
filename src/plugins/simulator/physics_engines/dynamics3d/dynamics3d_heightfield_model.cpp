/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_heightfield_model.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "dynamics3d_heightfield_model.h"
#include "dynamics3d_engine.h"

#include <btBulletDynamicsCommon.h>

namespace argos {

   /****************************************/
   /****************************************/

   CDynamics3DHeightFieldModel::CDynamics3DHeightFieldModel(CDynamics3DEngine& c_engine,
                                  CHeightFieldEntity& c_terrain) :
      CDynamics3DEntity(c_engine, c_terrain.GetEmbodiedEntity()),
      m_cTerrainEntity(c_terrain) {
      
      const CHeightFieldEntity::SHeightMap& sHeightMap = c_terrain.GetHeightMap();
      
      m_pcCollisionShape = new btHeightfieldTerrainShape(
         sHeightMap.GridSizeX, 
	      sHeightMap.GridSizeY,
			sHeightMap.Data.ToCArray(),
			sHeightMap.GridHeightScale, 
			sHeightMap.MinimumHeight, 
			sHeightMap.MaximumHeight,
			1,
			PHY_FLOAT, 
			false
	   );
	   
	   m_pcCollisionShape->setLocalScaling(sTerrainHeightMap.GridScaling.GetX()
	                                       sTerrainHeightMap.GridScaling.GetZ()
	                                       sTerrainHeightMap.GridScaling.GetY());

      m_pcMotionState = new btDefaultMotionState(btTransform(
         ARGoSToBullet(GetEmbodiedEntity().GetOrientation()),
         ARGoSToBullet(GetEmbodiedEntity().GetPosition())
      ));
      
      m_vecRigidBodies.push_back(new btRigidBody(btRigidBody::btRigidBodyConstructionInfo(
         0.0f, m_pcMotionState, m_pcCollisionShape, btVector3(0.0f,0.0f,0.0f))));
   }
   
   /****************************************/
   /****************************************/
   
   CDynamics3DHeightFieldModel::~CDynamics3DHeightFieldModel() {
      for(std::vector<btRigidBody*>::iterator it = m_vecRigidBodies.begin();
          it != m_vecRigidBodies.end();
          it++) {
         delete *it;
      }
      delete m_pcMotionState;
      delete m_pcCollisionShape;
   }
   
   /****************************************/
   /****************************************/

   bool CDynamics3DHeightFieldModel::CheckIntersectionWithRay(Real& f_t_on_ray,
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
  
   bool CDynamics3DHeightFieldModel::MoveTo(const CVector3& c_position,
                                     const CQuaternion& c_orientation,
                                     bool b_check_only) {
      
      LOG << "Move to location " << c_position << " requested" << std::endl;
      LOG << "Check only mode: " << (b_check_only?"enabled":"disabled") << std::endl;
      
      /* Create a transform to the new location and orientation */   
      btTransform cEntityTransform(ARGoSToBullet(c_orientation), ARGoSToBullet(c_position));
      
      /* Test if this region defined by the location and collision shape is occupied */
      bool bLocationOccupied = 
         m_cEngine.IsRegionOccupied(cEntityTransform, *m_pcCollisionShape);
         
      LOG << "CDynamics3DEngine::IsRegionOccupied returned: " << (bLocationOccupied?"true":"false") << std::endl;
         
      if(b_check_only == false && bLocationOccupied == false) {
         m_vecRigidBodies[0]->setWorldTransform(cEntityTransform);
         GetEmbodiedEntity().SetPosition(c_position);
         GetEmbodiedEntity().SetOrientation(c_orientation);
      }
      
      LOG << "Final location " << GetEmbodiedEntity().GetPosition() << std::endl;
      return !bLocationOccupied;
   }
   
   /****************************************/
   /****************************************/

   void CDynamics3DHeightFieldModel::Reset() {
      if(m_cBoxEntity.GetEmbodiedEntity().IsMovable()) {      
         
         /* Reset box position and orientation */
         m_pcMotionState->setWorldTransform(btTransform(
            ARGoSToBullet(GetEmbodiedEntity().GetInitOrientation()),
            ARGoSToBullet(GetEmbodiedEntity().GetInitPosition())
         ));
         
         m_vecRigidBodies[0]->setAngularVelocity(btVector3(0.0f, 0.0f, 0.0f));
         m_vecRigidBodies[0]->setLinearVelocity(btVector3(0.0f, 0.0f, 0.0f));
         m_vecRigidBodies[0]->clearForces();
      }
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_DYNAMICS3D_OPERATIONS_ON_ENTITY(CHeightFieldEntity, CDynamics3DHeightFieldModel);

   /****************************************/
   /****************************************/

}
