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

         //DEBUG
         btDefaultMotionState * pcMotionState = itBodyConfiguration->second->m_pcMotionState;

fprintf(stderr, "[INIT_DEBUG] %s/m_graphicsWorldTrans: position = [%.3f, %.3f, %.3f], rotation axis = [%.3f, %.3f, %.3f] & angle = %.3f\n", itBodyConfiguration->first.c_str(), pcMotionState->m_graphicsWorldTrans.getOrigin().getX(), pcMotionState->m_graphicsWorldTrans.getOrigin().getY(), pcMotionState->m_graphicsWorldTrans.getOrigin().getZ(), pcMotionState->m_graphicsWorldTrans.getRotation().getAxis().getX(), pcMotionState->m_graphicsWorldTrans.getRotation().getAxis().getY(), pcMotionState->m_graphicsWorldTrans.getRotation().getAxis().getZ(), pcMotionState->m_graphicsWorldTrans.getRotation().getAngle() * 57.2957795131f);

         fprintf(stderr, "[INIT_DEBUG] %s/m_centerOfMassOffset: position = [%.3f, %.3f, %.3f], rotation axis = [%.3f, %.3f, %.3f] & angle = %.3f\n", itBodyConfiguration->first.c_str(), pcMotionState->m_centerOfMassOffset.getOrigin().getX(), pcMotionState->m_centerOfMassOffset.getOrigin().getY(), pcMotionState->m_centerOfMassOffset.getOrigin().getZ(), pcMotionState->m_centerOfMassOffset.getRotation().getAxis().getX(), pcMotionState->m_centerOfMassOffset.getRotation().getAxis().getY(), pcMotionState->m_centerOfMassOffset.getRotation().getAxis().getZ(), pcMotionState->m_centerOfMassOffset.getRotation().getAngle() * 57.2957795131f);

         fprintf(stderr, "[INIT_DEBUG] %s/m_startWorldTrans:    position = [%.3f, %.3f, %.3f], rotation axis = [%.3f, %.3f, %.3f] & angle = %.3f\n", itBodyConfiguration->first.c_str(), pcMotionState->m_startWorldTrans.getOrigin().getX(), pcMotionState->m_startWorldTrans.getOrigin().getY(), pcMotionState->m_startWorldTrans.getOrigin().getZ(), pcMotionState->m_startWorldTrans.getRotation().getAxis().getX(), pcMotionState->m_startWorldTrans.getRotation().getAxis().getY(), pcMotionState->m_startWorldTrans.getRotation().getAxis().getZ(), pcMotionState->m_startWorldTrans.getRotation().getAngle() * 57.2957795131f);
  

         //DEBUG


      }

      // move model into the requested position
      //SetModelCoordinates(GetEmbodiedEntity().GetInitPosition(), GetEmoddiedEntity().GetInitOrientation());

      // Update the bodies inside the entity which have their positions driven by the physics engines
      UpdateEntityStatus();
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
         
         //@todo optimise by storing a pointer to the CPositionalEntity inside the SBodyConfiguration structure
         SBodyConfiguration* psBodyConfiguration = m_mapRobotBodyConfigurations[(*itBody)->GetId()];
         
         //@todo move this offset and transform logic inside the motion state
         btTransform cOffset(ARGoSToBullet((*itBody)->m_cOffsetOrientation), ARGoSToBullet((*itBody)->m_cOffsetPosition));
         
         // when updating the components we don't want to undo the offset!!
         const btTransform& cBodyUpdateTransform = psBodyConfiguration->m_pcMotionState->m_graphicsWorldTrans;
         //const btTransform& cBodyUpdateTransform = psBodyConfiguration->m_pcMotionState->m_graphicsWorldTrans * cOffset.inverse();

         fprintf(stderr, "[DEBUG] Position of %s in ARGoS *before* update\n", (*itBody)->GetId().c_str());

         CVector3 position;
         CVector3 axis;
         CRadians angle;

         (*itBody)->GetPositionalEntity().GetOrientation().ToAngleAxis(angle, axis);
         position = (*itBody)->GetPositionalEntity().GetPosition();

         fprintf(stderr, "absolute: position = [%.3f, %.3f, %.3f], rotation axis = [%.3f, %.3f, %.3f] & angle = %.3f\n", position.GetX(), position.GetY(), position.GetZ(), axis.GetX(), axis.GetY(), axis.GetZ(), ToDegrees(angle).GetValue());

         (*itBody)->m_cOffsetOrientation.ToAngleAxis(angle, axis);
         position = (*itBody)->m_cOffsetPosition;

         fprintf(stderr, "offset: position = [%.3f, %.3f, %.3f], rotation axis = [%.3f, %.3f, %.3f] & angle = %.3f\n", position.GetX(), position.GetY(), position.GetZ(), axis.GetX(), axis.GetY(), axis.GetZ(), ToDegrees(angle).GetValue());

         fprintf(stderr, "\n");

         btDefaultMotionState * pcMotionState = psBodyConfiguration->m_pcMotionState;
         
         fprintf(stderr, "[DEBUG] %s/m_graphicsWorldTrans: position = [%.3f, %.3f, %.3f], rotation axis = [%.3f, %.3f, %.3f] & angle = %.3f\n", (*itBody)->GetId().c_str(), pcMotionState->m_graphicsWorldTrans.getOrigin().getX(), pcMotionState->m_graphicsWorldTrans.getOrigin().getY(), pcMotionState->m_graphicsWorldTrans.getOrigin().getZ(), pcMotionState->m_graphicsWorldTrans.getRotation().getAxis().getX(), pcMotionState->m_graphicsWorldTrans.getRotation().getAxis().getY(), pcMotionState->m_graphicsWorldTrans.getRotation().getAxis().getZ(), pcMotionState->m_graphicsWorldTrans.getRotation().getAngle() * 57.2957795131f);

         fprintf(stderr, "[DEBUG] %s/m_centerOfMassOffset: position = [%.3f, %.3f, %.3f], rotation axis = [%.3f, %.3f, %.3f] & angle = %.3f\n", (*itBody)->GetId().c_str(), pcMotionState->m_centerOfMassOffset.getOrigin().getX(), pcMotionState->m_centerOfMassOffset.getOrigin().getY(), pcMotionState->m_centerOfMassOffset.getOrigin().getZ(), pcMotionState->m_centerOfMassOffset.getRotation().getAxis().getX(), pcMotionState->m_centerOfMassOffset.getRotation().getAxis().getY(), pcMotionState->m_centerOfMassOffset.getRotation().getAxis().getZ(), pcMotionState->m_centerOfMassOffset.getRotation().getAngle() * 57.2957795131f);

         fprintf(stderr, "[DEBUG] %s/m_startWorldTrans:    position = [%.3f, %.3f, %.3f], rotation axis = [%.3f, %.3f, %.3f] & angle = %.3f\n", (*itBody)->GetId().c_str(), pcMotionState->m_startWorldTrans.getOrigin().getX(), pcMotionState->m_startWorldTrans.getOrigin().getY(), pcMotionState->m_startWorldTrans.getOrigin().getZ(), pcMotionState->m_startWorldTrans.getRotation().getAxis().getX(), pcMotionState->m_startWorldTrans.getRotation().getAxis().getY(), pcMotionState->m_startWorldTrans.getRotation().getAxis().getZ(), pcMotionState->m_startWorldTrans.getRotation().getAngle() * 57.2957795131f);
         
         fprintf(stderr, "\n");

         fprintf(stderr, "[DEBUG] %s/cBodyUpdateTransform:    position = [%.3f, %.3f, %.3f], rotation axis = [%.3f, %.3f, %.3f] & angle = %.3f\n", (*itBody)->GetId().c_str(), cBodyUpdateTransform.getOrigin().getX(), cBodyUpdateTransform.getOrigin().getY(), cBodyUpdateTransform.getOrigin().getZ(), cBodyUpdateTransform.getRotation().getAxis().getX(), cBodyUpdateTransform.getRotation().getAxis().getY(), cBodyUpdateTransform.getRotation().getAxis().getZ(), cBodyUpdateTransform.getRotation().getAngle() * 57.2957795131f);

         (*itBody)->GetPositionalEntity().SetPosition(BulletToARGoS(cBodyUpdateTransform.getOrigin()));      
         (*itBody)->GetPositionalEntity().SetOrientation(BulletToARGoS(cBodyUpdateTransform.getRotation()));

         fprintf(stderr, "\n");

         fprintf(stderr, "[DEBUG] Position of %s in ARGoS *after* update\n", (*itBody)->GetId().c_str());

(*itBody)->GetPositionalEntity().GetOrientation().ToAngleAxis(angle, axis);
         position = (*itBody)->GetPositionalEntity().GetPosition();

         fprintf(stderr, "absolute: position = [%.3f, %.3f, %.3f], rotation axis = [%.3f, %.3f, %.3f] & angle = %.3f\n", position.GetX(), position.GetY(), position.GetZ(), axis.GetX(), axis.GetY(), axis.GetZ(), ToDegrees(angle).GetValue());

         (*itBody)->m_cOffsetOrientation.ToAngleAxis(angle, axis);
         position = (*itBody)->m_cOffsetPosition;

         fprintf(stderr, "offset: position = [%.3f, %.3f, %.3f], rotation axis = [%.3f, %.3f, %.3f] & angle = %.3f\n", position.GetX(), position.GetY(), position.GetZ(), axis.GetX(), axis.GetY(), axis.GetZ(), ToDegrees(angle).GetValue());

         fprintf(stderr, "\n");

         fprintf(stderr, "\n");

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

