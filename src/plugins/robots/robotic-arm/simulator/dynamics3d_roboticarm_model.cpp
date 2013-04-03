/**
 * @file <argos3/plugins/robots/robotic-arm/simulator/dynamics3d_roboticarm_model.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "dynamics3d_roboticarm_model.h"
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_engine.h>

namespace argos {
   
   /****************************************/
   /****************************************/
   
   const static btTransform shift(btQuaternion(0,0,0,1), btVector3(0.0,0.05,0.0));

   const static Real MOUNTING_POINT_MASS = 0.5f;

   CDynamics3DRoboticArmModel::CDynamics3DRoboticArmModel(CDynamics3DEngine& c_engine,
                                                            CRoboticArmEntity& c_entity) :
      CDynamics3DModel(c_engine, c_entity.GetEmbodiedEntity()),
      m_cRoboticArmEntity(c_entity) {
      
      btVector3 cInertia;
      
      btTransform cModelTransform(ARGoSToBullet(GetEmbodiedEntity().GetOrientation()),
                                   ARGoSToBullet(GetEmbodiedEntity().GetPosition()));
      
      m_pcMountingPointCollisionShape = new btSphereShape(0.050f);
      
      m_pcMountingPointCollisionShape->calculateLocalInertia(MOUNTING_POINT_MASS, cInertia);
      
      m_pcMountingPointMotionState = new btDefaultMotionState(cModelTransform * shift);
      
      m_pcMountingPointRigidBody = new btRigidBody(btRigidBody::btRigidBodyConstructionInfo(
         MOUNTING_POINT_MASS, m_pcMountingPointMotionState, m_pcMountingPointCollisionShape, cInertia));
         
      m_mapLocalRigidBodies["mounting_point"] = m_pcMountingPointRigidBody;
      
   }

   /****************************************/
   /****************************************/

   CDynamics3DRoboticArmModel::~CDynamics3DRoboticArmModel() {
      delete m_pcMountingPointCollisionShape;
      delete m_pcMountingPointMotionState;
      delete m_pcMountingPointRigidBody;
   }

   /****************************************/
   /****************************************/

   void CDynamics3DRoboticArmModel::UpdateEntityStatus() {
      /* Update roboticarm position and orientation */
      btTransform cModelTransform;
      m_pcMountingPointMotionState->getWorldTransform(cModelTransform);
      cModelTransform = shift.inverse() * cModelTransform;
      GetEmbodiedEntity().SetPosition(BulletToARGoS(cModelTransform.getOrigin()));
      GetEmbodiedEntity().SetOrientation(BulletToARGoS(cModelTransform.getRotation()));
      
      /* Update components */
      m_cRoboticArmEntity.UpdateComponents();
   }

   /****************************************/
   /****************************************/

   void CDynamics3DRoboticArmModel::UpdateFromEntityStatus() {}

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_DYNAMICS3D_OPERATIONS_ON_ENTITY(CRoboticArmEntity, CDynamics3DRoboticArmModel);

   /****************************************/
   /****************************************/

}

