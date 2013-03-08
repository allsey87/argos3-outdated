/**
 * @file <argos3/plugins/robots/robotic-arm/simulator/dynamics3d_roboticarm_entity.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "dynamics3d_roboticarm_entity.h"
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_engine.h>

namespace argos {
   
   /****************************************/
   /****************************************/
   
   const static btTransform shift(btQuaternion(0,0,0,1), btVector3(0.0,0.05,0.0));

   const static Real MOUNTING_POINT_MASS = 0.5f;

   CDynamics3DRoboticArmEntity::CDynamics3DRoboticArmEntity(CDynamics3DEngine& c_engine,
                                                            CRoboticArmEntity& c_entity) :
      CDynamics3DEntity(c_engine, c_entity.GetEmbodiedEntity()),
      m_cRoboticArmEntity(c_entity) {
      
      btVector3 cInertia;
      
      btTransform cEntityTransform(ARGoSToBullet(GetEmbodiedEntity().GetOrientation()),
                                   ARGoSToBullet(GetEmbodiedEntity().GetPosition()));
      
      m_pcMountingPointCollisionShape = new btSphereShape(0.050f);
      
      m_pcMountingPointCollisionShape->calculateLocalInertia(MOUNTING_POINT_MASS, cInertia);
      
      m_pcMountingPointMotionState = new btDefaultMotionState(cEntityTransform * shift);
      
      m_pcMountingPointRigidBody = new btRigidBody(btRigidBody::btRigidBodyConstructionInfo(
         MOUNTING_POINT_MASS, m_pcMountingPointMotionState, m_pcMountingPointCollisionShape, cInertia));
         
      m_vecLocalRigidBodies.push_back(m_pcMountingPointRigidBody);

      if(c_entity.IsAttachedToSomething()) {
         /* @todo Missing check that attachee belongs to the same physics engine as the robotic arm */
         CEmbodiedEntity& cAttachee = c_entity.GetAttachee();
         /* Get reference to body in this engine */
         /* Create point constraint */
         /* Profit */
      }
      
   }

   /****************************************/
   /****************************************/

   CDynamics3DRoboticArmEntity::~CDynamics3DRoboticArmEntity() {
      delete m_pcMountingPointCollisionShape;
      delete m_pcMountingPointMotionState;
      delete m_pcMountingPointRigidBody;
   }

   /****************************************/
   /****************************************/

   bool CDynamics3DRoboticArmEntity::CheckIntersectionWithRay(Real& f_t_on_ray,
                                                           const CRay3& c_ray) const {
      return false;
   }

   /****************************************/
   /****************************************/

   bool CDynamics3DRoboticArmEntity::MoveTo(const CVector3& c_position,
                                         const CQuaternion& c_orientation,
                                         bool b_check_only) {
      return false;
   }

   /****************************************/
   /****************************************/

   void CDynamics3DRoboticArmEntity::Reset() {
      btTransform cResetPosition(ARGoSToBullet(GetEmbodiedEntity().GetInitOrientation()),
                                 ARGoSToBullet(GetEmbodiedEntity().GetInitPosition()));
      m_pcMountingPointRigidBody->setWorldTransform(cResetPosition * shift);
      
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

   void CDynamics3DRoboticArmEntity::CalculateBoundingBox() {
      /* @todo Implement CDynamics3DBoxEntity::CalculateBoundingBox() */
   }

   /****************************************/
   /****************************************/

   void CDynamics3DRoboticArmEntity::UpdateEntityStatus() {
      /* Update roboticarm position and orientation */
      btTransform cEntityTransform;
      m_pcMountingPointMotionState->getWorldTransform(cEntityTransform);
      cEntityTransform = shift.inverse() * cEntityTransform;
      GetEmbodiedEntity().SetPosition(BulletToARGoS(cEntityTransform.getOrigin()));
      GetEmbodiedEntity().SetOrientation(BulletToARGoS(cEntityTransform.getRotation()));
      
      /* Update components */
      m_cRoboticArmEntity.UpdateComponents();
   }

   /****************************************/
   /****************************************/

   void CDynamics3DRoboticArmEntity::UpdateFromEntityStatus() {}

   /****************************************/
   /****************************************/

   bool CDynamics3DRoboticArmEntity::IsCollidingWithSomething() const {
      /* @todo Implement CDynamics3DBoxEntity::IsCollidingWithSomething() */
      return false;
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_DYNAMICS3D_OPERATIONS_ON_ENTITY(CRoboticArmEntity, CDynamics3DRoboticArmEntity);

   /****************************************/
   /****************************************/

}

