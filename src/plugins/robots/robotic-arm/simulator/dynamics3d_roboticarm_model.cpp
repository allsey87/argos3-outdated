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
   
   const static btTransform shift(btQuaternion(0,0,0,1), btVector3(0.0,0.1,0.0));

   const static Real LINK_MASS = 0.5f;

   CDynamics3DRoboticArmModel::CDynamics3DRoboticArmModel(CDynamics3DEngine& c_engine,
                                                            CRoboticArmEntity& c_entity) :
      CDynamics3DModel(c_engine, c_entity.GetEmbodiedEntity()),
      m_cRoboticArmEntity(c_entity),
      m_cLinkEquippedEntity(c_entity.GetLinkEquippedEntity()) {
      
      btVector3 cInertia;
      
      btTransform cModelTransform(ARGoSToBullet(GetEmbodiedEntity().GetOrientation()),
                                  ARGoSToBullet(GetEmbodiedEntity().GetPosition()));
      
      /* @todo the use of iterators here is becoming messy, switch to using UInt32 */
      for(CLinkEntity::TList::iterator itLink = m_cLinkEquippedEntity.GetAllLinks().begin();
          itLink != m_cLinkEquippedEntity.GetAllLinks().end();
          itLink++) {
         
         m_vecLinkCollisionShapes.push_back(new btBoxShape(btVector3(0.025f, 0.2f, 0.025f)));

         m_vecLinkCollisionShapes.back()->calculateLocalInertia(LINK_MASS, cInertia);
         
         if(m_vecLinkMotionStates.empty()) {
            /* this is the first link in the kinematic chain */
            m_vecLinkMotionStates.push_back(new btDefaultMotionState(cModelTransform * shift));
         }
         else {
            /* This link follows another link, use the previous link's transform */
            btTransform cLastLinkTransform;
            m_vecLinkMotionStates.back()->getWorldTransform(cLastLinkTransform);
            m_vecLinkMotionStates.push_back(new btDefaultMotionState(cLastLinkTransform * shift));
            /* the shift in these statements is compensating for the distance between links, not the offset from the ground. only the first element contains this offset! */
         }

         m_vecLinkRigidBodies.push_back(new btRigidBody(btRigidBody::btRigidBodyConstructionInfo(
            LINK_MASS, m_vecLinkMotionStates.back(), m_vecLinkCollisionShapes.back(), cInertia)));

         /* There was a link before us! - join the last two bodies with a hinge constraint*/
         if(m_vecLinkRigidBodies.size() > 1) {
            m_vecLinkConstraints.push_back(new btHingeConstraint(**(m_vecLinkRigidBodies.end() - 2),
                                                                 **(m_vecLinkRigidBodies.end() - 1),
                                                                 btVector3(0.0f,
                                                                           0.1f,
                                                                           0.0f),
                                                                 btVector3(0.0f,
                                                                           -0.1,
                                                                           0.0f),
                                                                 btVector3(0,0,1),
                                                                 btVector3(0,0,1)));
         }

      }

      /* @todo these data structures will change to maps */
      m_vecLocalRigidBodies.assign(m_vecLinkRigidBodies.begin(), m_vecLinkRigidBodies.end());
      m_vecLocalConstraints.assign(m_vecLinkConstraints.begin(), m_vecLinkConstraints.end());
     
      
   }

   /****************************************/
   /****************************************/

   CDynamics3DRoboticArmModel::~CDynamics3DRoboticArmModel() {
      std::vector<btBoxShape*>::iterator itCollisionShape;
      std::vector<btDefaultMotionState*>::iterator itMotionState;
      std::vector<btRigidBody*>::iterator itRigidBody;
      std::vector<btHingeConstraint*>::iterator itConstraint;

      for(itConstraint = m_vecLinkConstraints.begin();
          itConstraint != m_vecLinkConstraints.end();
          itConstraint++) {
         delete *itConstraint;
      }

      for(itRigidBody = m_vecLinkRigidBodies.begin();
          itRigidBody != m_vecLinkRigidBodies.end();
          itRigidBody++) {
         delete *itRigidBody;
      }

      for(itMotionState = m_vecLinkMotionStates.begin();
          itMotionState != m_vecLinkMotionStates.end();
          itMotionState++) {
         delete *itMotionState;
      }      

      for(itCollisionShape = m_vecLinkCollisionShapes.begin();
          itCollisionShape != m_vecLinkCollisionShapes.end();
          itCollisionShape++) {
         delete *itCollisionShape;
      }
   }

   /****************************************/
   /****************************************/

   bool CDynamics3DRoboticArmModel::CheckIntersectionWithRay(Real& f_t_on_ray,
                                                           const CRay3& c_ray) const {
      /* @todo implement CDynamics3DRoboticArmModel::CheckIntersectionWithRay */
      return false;
   }

   /****************************************/
   /****************************************/

   bool CDynamics3DRoboticArmModel::MoveTo(const CVector3& c_position,
                                         const CQuaternion& c_orientation,
                                         bool b_check_only) {
      /* @todo implement CDynamics3DRoboticArmModel::MoveTo */
      return false;
   }

   /****************************************/
   /****************************************/

   void CDynamics3DRoboticArmModel::Reset() {
      btTransform cResetPosition(ARGoSToBullet(GetEmbodiedEntity().GetInitOrientation()),
                                 ARGoSToBullet(GetEmbodiedEntity().GetInitPosition()));
      //m_pcMountingPointRigidBody->setWorldTransform(cResetPosition * shift);
      
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

   void CDynamics3DRoboticArmModel::CalculateBoundingBox() {
      /* @todo Implement CDynamics3DBoxEntity::CalculateBoundingBox() */
   }

   /****************************************/
   /****************************************/

   void CDynamics3DRoboticArmModel::UpdateEntityStatus() {
      /* Update roboticarm position and orientation */
      btTransform cLinkTransform;
      //m_pcMountingPointMotionState->getWorldTransform(cModelTransform);
      for(UInt32 i = 0; i < m_cLinkEquippedEntity.GetAllLinks().size(); i++) {
         m_vecLinkMotionStates[i]->getWorldTransform(cLinkTransform);
         CLinkEntity& cLink =  m_cLinkEquippedEntity.GetLink(i);
         cLink.GetPositionalEntity().SetPosition(BulletToARGoS(cLinkTransform.getOrigin()));      
         cLink.GetPositionalEntity().SetOrientation(BulletToARGoS(cLinkTransform.getRotation()));
      }

      /* @todo what position should/must the emboddied entity contain for a multi body non symmetric system? */
      //cModelTransform = shift.inverse() * cModelTransform;
      //GetEmbodiedEntity().SetPosition();
      //GetEmbodiedEntity().SetOrientation();
      
      /* Update components */
      m_cRoboticArmEntity.UpdateComponents();
   }

   /****************************************/
   /****************************************/

   void CDynamics3DRoboticArmModel::UpdateFromEntityStatus() {}

   /****************************************/
   /****************************************/

   bool CDynamics3DRoboticArmModel::IsCollidingWithSomething() const {
      /* @todo Implement CDynamics3DBoxEntity::IsCollidingWithSomething() */
      return false;
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_DYNAMICS3D_OPERATIONS_ON_ENTITY(CRoboticArmEntity, CDynamics3DRoboticArmModel);

   /****************************************/
   /****************************************/

}

