/**
 * @file <argos3/plugins/robots/robot/simulator/dynamics3d_robot_model.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "dynamics3d_robot_model.h"
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_engine.h>

#include <argos3/plugins/robots/robot/simulator/joint_equipped_entity.h>
#include <argos3/plugins/robots/robot/simulator/body_equipped_entity.h>

namespace argos {
   
   /****************************************/
   /****************************************/

   CDynamics3DRobotModel::CDynamics3DRobotModel(CDynamics3DEngine& c_engine,
                                                CRobotEntity& c_entity) :
      CDynamics3DModel(c_engine, c_entity.GetEmbodiedEntity()),
      m_cRobotEntity(c_entity),
      m_cBodyEquippedEntity(c_entity.GetBodyEquippedEntity()),
      m_cJointEquippedEntity(c_entity.GetJointEquippedEntity()) {
      
      btVector3 cInertia;
      
      btTransform cModelTransform(ARGoSToBullet(GetEmbodiedEntity().GetOrientation()),
                                  ARGoSToBullet(GetEmbodiedEntity().GetPosition()));
      
      for(CBodyEntity::TList::iterator itBody = m_cBodyEquippedEntity.GetAllBodies().begin();
          itBody != m_cBodyEquippedEntity.GetAllBodies().end();
          ++itBody) {
         
         btVector3 bodyHalfExtents((*itBody)->GetSize().GetX() * 0.5f,
                                   (*itBody)->GetSize().GetZ() * 0.5f,
                                   (*itBody)->GetSize().GetY() * 0.5f);

         SBodyConfiguration* psBodyConfiguration = new SBodyConfiguration;

         psBodyConfiguration->m_pcCollisionShape = new btBoxShape(bodyHalfExtents);
         psBodyConfiguration->m_pcCollisionShape->calculateLocalInertia((*itBody)->GetMass(), cInertia);
         
         btTransform cOffset(ARGoSToBullet((*itBody)->m_cOffsetOrientation), ARGoSToBullet((*itBody)->m_cOffsetPosition));
         
         psBodyConfiguration->m_pcMotionState = new btDefaultMotionState(cModelTransform * cOffset, btTransform(
            btQuaternion(0,0,0,1),
            btVector3(0, -bodyHalfExtents.getY(), 0)));
         
         psBodyConfiguration->m_pcRigidBody = new btRigidBody(btRigidBody::btRigidBodyConstructionInfo(
            (*itBody)->GetMass(), psBodyConfiguration->m_pcMotionState, 
            psBodyConfiguration->m_pcCollisionShape, cInertia));

         m_mapRobotBodyConfigurations[(*itBody)->GetId()] = psBodyConfiguration;
      }

      for(CJointEntity::TList::iterator itJoint = m_cJointEquippedEntity.GetAllJoints().begin();
          itJoint != m_cJointEquippedEntity.GetAllJoints().end();
          ++itJoint) {
         
         //CBodyEntity::TList& tConnectedBodies = (*itJoint)->GetAllConnectedBodies();

         //@todo check joint type!

         //@todo remove these public members, find clean solutions for parameterising joints
         //const std::vector<CVector3>& vecRotationAxes = (*itJoint)->m_vecRotationAxes;
         //const std::vector<CVector3>& vecRotationPoints = (*itJoint)->m_vecRotationPoints;
      }

      // transfer the rigid body pointers to the collection in the base class
      std::map<std::string, SBodyConfiguration*>::iterator itBodyConfiguration;

      for(itBodyConfiguration = m_mapRobotBodyConfigurations.begin();
          itBodyConfiguration != m_mapRobotBodyConfigurations.end();
          ++itBodyConfiguration) {
         
         m_mapLocalRigidBodies[itBodyConfiguration->first] = itBodyConfiguration->second->m_pcRigidBody;
      }

      // move model into the requested position
      //SetModelCoordinates(GetEmbodiedEntity().GetInitPosition(), GetEmoddiedEntity().GetInitOrientation());
      
   }

   /****************************************/
   /****************************************/

   CDynamics3DRobotModel::~CDynamics3DRobotModel() {
      std::map<std::string, SBodyConfiguration*>::iterator itBodyConfiguration;
      std::map<std::string, btHingeConstraint*>::iterator itConstraint;

      for(itConstraint = m_mapRobotConstraints.begin();
          itConstraint != m_mapRobotConstraints.end();
          ++itConstraint) {
         delete itConstraint->second;
      }
      
      for(itBodyConfiguration = m_mapRobotBodyConfigurations.begin();
          itBodyConfiguration != m_mapRobotBodyConfigurations.end();
          ++itBodyConfiguration) {
         delete itBodyConfiguration->second->m_pcRigidBody;
         delete itBodyConfiguration->second->m_pcMotionState;
         delete itBodyConfiguration->second->m_pcCollisionShape;
         delete itBodyConfiguration->second;
      }
   }

   /****************************************/
   /****************************************/

   void CDynamics3DRobotModel::UpdateEntityStatus() {
      /* Update robot position and orientation */
      btTransform cBodyTransform;
      
      for(CBodyEntity::TList::iterator itBody = m_cBodyEquippedEntity.GetAllBodies().begin();
          itBody != m_cBodyEquippedEntity.GetAllBodies().end();
          ++itBody) {
         
         SBodyConfiguration* psBodyConfiguration = m_mapRobotBodyConfigurations[(*itBody)->GetId()];
         
         //@todo move this offset and transform logic inside the motion state
         btTransform cOffset(ARGoSToBullet((*itBody)->m_cOffsetOrientation), ARGoSToBullet((*itBody)->m_cOffsetPosition));
         
         const btTransform& cBodyUpdateTransform = psBodyConfiguration->m_pcMotionState->m_graphicsWorldTrans * cOffset.inverse();

         (*itBody)->GetPositionalEntity().SetPosition(BulletToARGoS(cBodyUpdateTransform.getOrigin()));      
         (*itBody)->GetPositionalEntity().SetOrientation(BulletToARGoS(cBodyUpdateTransform.getRotation()));
      }

      /* Update components */
      m_cRobotEntity.UpdateComponents();
   }

   /****************************************/
   /****************************************/

   void CDynamics3DRobotModel::UpdateFromEntityStatus() {}

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_DYNAMICS3D_OPERATIONS_ON_ENTITY(CRobotEntity, CDynamics3DRobotModel);

   /****************************************/
   /****************************************/

}

