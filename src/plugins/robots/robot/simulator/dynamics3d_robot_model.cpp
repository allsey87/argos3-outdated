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
#include <argos3/plugins/robots/robot/simulator/frame_equipped_entity.h>

#include <argos3/plugins/robots/robot/simulator/geometry3.h>

#include <argos3/core/utility/math/matrix/rotationmatrix3.h>

namespace argos {
   
   /****************************************/
   /****************************************/

   CDynamics3DRobotModel::CDynamics3DRobotModel(CDynamics3DEngine& c_engine,
                                                CRobotEntity& c_entity) :
      CDynamics3DModel(c_engine, c_entity.GetEmbodiedEntity(), c_entity.GetId()),
      m_cRobotEntity(c_entity),
      m_cBodyEquippedEntity(c_entity.GetBodyEquippedEntity()),
      m_cJointEquippedEntity(c_entity.GetJointEquippedEntity()) {
      
      for(CBodyEntity::TList::iterator itBody = m_cBodyEquippedEntity.GetAllBodies().begin();
          itBody != m_cBodyEquippedEntity.GetAllBodies().end();
          ++itBody) {

         btCollisionShape* pcShape = NULL;
         btVector3 cExtents((*itBody)->GetGeometry().GetExtents().GetX(),
                            (*itBody)->GetGeometry().GetExtents().GetZ(),
                            (*itBody)->GetGeometry().GetExtents().GetY());
         /* check the tag to determine which shape manager to use */
         switch((*itBody)->GetGeometry().GetTag()) {
         case CGeometry3::BOX:
            pcShape = m_cBoxShapeManager.RequestBoxShape(cExtents * 0.5f);
            break;
         case CGeometry3::CYLINDER:
            pcShape = m_cCylinderShapeManager.RequestCylinderShape(cExtents * 0.5f);
            break;
         case CGeometry3::SPHERE:
            /* we could dynamically cast this geometry to a sphere and take the radius
             * directly, however this is more efficient */
            pcShape = m_cSphereShapeManager.RequestSphereShape(cExtents.getX() * 0.5f);
            break;
         }
         btTransform cPositionalOffset(ARGoSToBullet((*itBody)->GetOffsetPositionalEntity().GetOrientation()),
                                       ARGoSToBullet((*itBody)->GetOffsetPositionalEntity().GetPosition()));
         btTransform cGeometricOffset(btQuaternion(0,0,0,1),
                                      btVector3(0, -cExtents.getY() * 0.5f, 0));
         m_vecLocalBodies.push_back(new CDynamics3DBody((*itBody)->GetId(), 
                                                        pcShape, 
                                                        cPositionalOffset,
                                                        cGeometricOffset,
                                                        (*itBody)->GetMass()));
      }

      for(CJointEntity::TList::iterator itJoint = m_cJointEquippedEntity.GetAllJoints().begin();
          itJoint != m_cJointEquippedEntity.GetAllJoints().end();
          ++itJoint) {
         CFrameEntity::TList& tFrames = (*itJoint)->GetFrameEquippedEntity().GetAllFrames();
         if(tFrames.size() != 2) {
            THROW_ARGOSEXCEPTION("This version of the Dynamics3D plugin only allows joints between two bodies");
         }
         CDynamics3DBody::TVector::iterator itDyn3dBody0 = std::find(m_vecLocalBodies.begin(),
                                                                     m_vecLocalBodies.end(),
                                                                     tFrames[0]->GetBodyEntity().GetId());
         if(itDyn3dBody0 == m_vecLocalBodies.end()) {
            THROW_ARGOSEXCEPTION("Unable to find referenced body \"" <<
                                 tFrames[0]->GetBodyEntity().GetId() << "\"." );
         }
         CDynamics3DBody::TVector::iterator itDyn3dBody1 = std::find(m_vecLocalBodies.begin(),
                                                                     m_vecLocalBodies.end(),
                                                                     tFrames[1]->GetBodyEntity().GetId());
         if(itDyn3dBody1 == m_vecLocalBodies.end()) {
            THROW_ARGOSEXCEPTION("Unable to find referenced body \"" <<
                                 tFrames[1]->GetBodyEntity().GetId() << "\"." );
         }

         /* Get the frames in each respective bodies */
         btTransform cFrameOriginInBody0 = (*itDyn3dBody0)->GetGeometricOffset() *
            btTransform(ARGoSToBullet(tFrames[0]->GetPositionalEntity().GetOrientation()),
                        ARGoSToBullet(tFrames[0]->GetPositionalEntity().GetPosition()));         
         btTransform cFrameOriginInBody1 = (*itDyn3dBody1)->GetGeometricOffset() *
            btTransform(ARGoSToBullet(tFrames[1]->GetPositionalEntity().GetOrientation()),
                        ARGoSToBullet(tFrames[1]->GetPositionalEntity().GetPosition()));
         

         /* Get the limits of the joint - we must manually swap Z and Y here! */
         /* linear */
         btVector3 cLinearLowerLimit((*itJoint)->GetAxisLinearX().m_bUnconstrained ?
                                     1.0f : (*itJoint)->GetAxisLinearX().m_cLimits.GetMin(),
                                     (*itJoint)->GetAxisLinearZ().m_bUnconstrained ?
                                     1.0f : (*itJoint)->GetAxisLinearZ().m_cLimits.GetMin(),
                                     (*itJoint)->GetAxisLinearY().m_bUnconstrained ?
                                     1.0f : (*itJoint)->GetAxisLinearY().m_cLimits.GetMin());
         btVector3 cLinearUpperLimit((*itJoint)->GetAxisLinearX().m_bUnconstrained ?
                                     -1.0f : (*itJoint)->GetAxisLinearX().m_cLimits.GetMax(),
                                     (*itJoint)->GetAxisLinearZ().m_bUnconstrained ?
                                     -1.0f : (*itJoint)->GetAxisLinearZ().m_cLimits.GetMax(),
                                     (*itJoint)->GetAxisLinearY().m_bUnconstrained ?
                                     -1.0f : (*itJoint)->GetAxisLinearY().m_cLimits.GetMax());               
         /* angular */
         btVector3 cAngularLowerLimit((*itJoint)->GetAxisAngularX().m_bUnconstrained ?
                                      1.0f : (*itJoint)->GetAxisAngularX().m_cLimits.GetMin().GetValue(),
                                      (*itJoint)->GetAxisAngularZ().m_bUnconstrained ?
                                      1.0f : (*itJoint)->GetAxisAngularZ().m_cLimits.GetMin().GetValue(),
                                      (*itJoint)->GetAxisAngularY().m_bUnconstrained ?
                                      1.0f : (*itJoint)->GetAxisAngularY().m_cLimits.GetMin().GetValue());
         btVector3 cAngularUpperLimit((*itJoint)->GetAxisAngularX().m_bUnconstrained ?
                                      -1.0f : (*itJoint)->GetAxisAngularX().m_cLimits.GetMax().GetValue(),
                                      (*itJoint)->GetAxisAngularZ().m_bUnconstrained ?
                                      -1.0f : (*itJoint)->GetAxisAngularZ().m_cLimits.GetMax().GetValue(),
                                      (*itJoint)->GetAxisAngularY().m_bUnconstrained ?
                                      -1.0f : (*itJoint)->GetAxisAngularY().m_cLimits.GetMax().GetValue());

         CDynamics3DJoint::SJointActuators sLinearActuators(
            CDynamics3DJoint::SJointActuators::SActuator((*itJoint)->GetAxisLinearX().m_bActuatorEnabled,
                                                         (*itJoint)->GetAxisLinearX().m_fActuatorForce,
                                                         (*itJoint)->GetAxisLinearX().m_fActuatorTargetVelocity),
            CDynamics3DJoint::SJointActuators::SActuator((*itJoint)->GetAxisLinearZ().m_bActuatorEnabled,
                                                         (*itJoint)->GetAxisLinearZ().m_fActuatorForce,
                                                         (*itJoint)->GetAxisLinearZ().m_fActuatorTargetVelocity),
            CDynamics3DJoint::SJointActuators::SActuator((*itJoint)->GetAxisLinearY().m_bActuatorEnabled,
                                                         (*itJoint)->GetAxisLinearY().m_fActuatorForce,
                                                         (*itJoint)->GetAxisLinearY().m_fActuatorTargetVelocity));       
         CDynamics3DJoint::SJointActuators sAngularActuators(
            CDynamics3DJoint::SJointActuators::SActuator((*itJoint)->GetAxisAngularX().m_bActuatorEnabled,
                                                         (*itJoint)->GetAxisAngularX().m_fActuatorForce,
                                                         (*itJoint)->GetAxisAngularX().m_fActuatorTargetVelocity),
            CDynamics3DJoint::SJointActuators::SActuator((*itJoint)->GetAxisAngularZ().m_bActuatorEnabled,
                                                         (*itJoint)->GetAxisAngularZ().m_fActuatorForce,
                                                         (*itJoint)->GetAxisAngularZ().m_fActuatorTargetVelocity),
            CDynamics3DJoint::SJointActuators::SActuator((*itJoint)->GetAxisAngularY().m_bActuatorEnabled,
                                                         (*itJoint)->GetAxisAngularY().m_fActuatorForce,
                                                         (*itJoint)->GetAxisAngularY().m_fActuatorTargetVelocity));
         
         fprintf(stderr, "Joint Limits for %s\n", (*itJoint)->GetId().c_str());
         
         fprintf(stderr, "cLinearLowerLimit = [%.3f, %.3f, %.3f]\n",
                 cLinearLowerLimit.getX(),
                 cLinearLowerLimit.getY(),
                 cLinearLowerLimit.getZ());
         fprintf(stderr, "cLinearUpperLimit = [%.3f, %.3f, %.3f]\n",
                 cLinearUpperLimit.getX(),
                 cLinearUpperLimit.getY(),
                 cLinearUpperLimit.getZ());
         fprintf(stderr, "cAngularLowerLimit = [%.3f, %.3f, %.3f]\n",
                 cAngularLowerLimit.getX(),
                 cAngularLowerLimit.getY(),
                 cAngularLowerLimit.getZ());
         fprintf(stderr, "cAngularUpperLimit = [%.3f, %.3f, %.3f]\n",
                 cAngularUpperLimit.getX(),
                 cAngularUpperLimit.getY(),
                 cAngularUpperLimit.getZ());
         

         fprintf(stderr, "sAngularActuators = x[%s, %.3f, %.3f], y[%s, %.3f, %.3f], z[%s, %.3f, %.3f]\n", 
                 sAngularActuators.m_sX.m_bEnabled?"true":"false",
                 sAngularActuators.m_sX.m_fForce,
                 sAngularActuators.m_sX.m_fTargetVelocity,
                 sAngularActuators.m_sY.m_bEnabled?"true":"false",
                 sAngularActuators.m_sY.m_fForce,
                 sAngularActuators.m_sY.m_fTargetVelocity,
                 sAngularActuators.m_sZ.m_bEnabled?"true":"false",
                 sAngularActuators.m_sZ.m_fForce,
                 sAngularActuators.m_sZ.m_fTargetVelocity);


         /* create the joint */
         m_vecLocalJoints.push_back(new CDynamics3DJoint((*itJoint)->GetId(),
                                                         **itDyn3dBody0,
                                                         **itDyn3dBody1,
                                                         cFrameOriginInBody0,
                                                         cFrameOriginInBody1,
                                                         CDynamics3DJoint::SJointLimits(cLinearLowerLimit, cLinearUpperLimit),
                                                         CDynamics3DJoint::SJointLimits(cAngularLowerLimit, cAngularUpperLimit),
                                                         sLinearActuators,
                                                         sAngularActuators,
                                                         true,
                                                         (*itJoint)->GetDisableLinkedBodyCollisions()));
      }
      SetModelCoordinates(btTransform(ARGoSToBullet(GetEmbodiedEntity().GetOrientation()),
                                      ARGoSToBullet(GetEmbodiedEntity().GetPosition())));
      /* Update the bodies inside the entity which have their positions driven by the physics engines */
      UpdateEntityStatus();
   }

   /****************************************/
   /****************************************/

   CDynamics3DRobotModel::~CDynamics3DRobotModel() {

      //@todo clean up collision shapes?

      /*
      CDynamics3DBody::TVector::iterator itBody;
      std::vector<SConstraint>::iterator itConstraint;

      for(itConstraint = m_vecLocalConstraints.begin();
          itConstraint != m_vecLocalConstraints.end();
          ++itConstraint) {
         delete itConstraint->m_pcConstraint;
      }
      
      for(itBody = m_vecLocalBodies.begin();
          itBody != m_vecLocalBodies.end();
          ++itBody) {
         delete itBody->second->m_pcCollisionShape;
      }
      */
   }

   /****************************************/
   /****************************************/

   void CDynamics3DRobotModel::UpdateEntityStatus() {
      /* Update robot position and orientation */
      btTransform cBodyTransform;
      

      // Update the position of the bodies
      // loop over the SBodyConfigurations instead? link to positional entities on init?
      for(CBodyEntity::TList::iterator itBody = m_cBodyEquippedEntity.GetAllBodies().begin();
          itBody != m_cBodyEquippedEntity.GetAllBodies().end();
          ++itBody) {
         
         //@todo optimise by storing a pointer to the CPositionalEntity inside the SBodyConfiguration structure
         CDynamics3DBody& sBodyConfiguration = **std::find(m_vecLocalBodies.begin(),
                                                                 m_vecLocalBodies.end(),
                                                                 (*itBody)->GetId());
         
         sBodyConfiguration.ActivateRigidBody();

         //@todo move this offset and transform logic inside the motion state
         //btTransform cOffset(ARGoSToBullet((*itBody)->m_cOffsetOrientation), ARGoSToBullet((*itBody)->m_cOffsetPosition));
         
         // when updating the components we don't want to undo the offset!!
         const btTransform& cBodyUpdateTransform = sBodyConfiguration.GetMotionStateTransform();
         //const btTransform& cBodyUpdateTransform = psBodyConfiguration->m_pcMotionState->m_graphicsWorldTrans * cOffset.inverse();
         /*
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
         */
         (*itBody)->GetPositionalEntity().SetPosition(BulletToARGoS(cBodyUpdateTransform.getOrigin()));      
         (*itBody)->GetPositionalEntity().SetOrientation(BulletToARGoS(cBodyUpdateTransform.getRotation()));
         /*
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
         */

      }
              
      const btTransform& cEntityUpdateTransform = GetModelCoordinates();
      
      GetEmbodiedEntity().SetPosition(BulletToARGoS(cEntityUpdateTransform.getOrigin()));
      GetEmbodiedEntity().SetOrientation(BulletToARGoS(cEntityUpdateTransform.getRotation()));

      /* Update components */
      m_cRobotEntity.UpdateComponents();


      //MORE DEBUGGING
      /*CVector3 position;
      CVector3 axis;
      CRadians angle;

      GetEmbodiedEntity().GetOrientation().ToAngleAxis(angle, axis);
      position = GetEmbodiedEntity().GetPosition();

      fprintf(stderr, "%s position = [%.3f, %.3f, %.3f], rotation axis = [%.3f, %.3f, %.3f] & angle = [%.3f]\n", m_cRobotEntity.GetId().c_str(), position.GetX(), position.GetY(), position.GetZ(), axis.GetX(), axis.GetY(), axis.GetZ(), ToDegrees(angle).GetValue());
      */
   }

   /****************************************/
   /****************************************/

   void CDynamics3DRobotModel::UpdateFromEntityStatus() {
      for(CJointEntity::TList::iterator itJoint = m_cJointEquippedEntity.GetAllJoints().begin();
          itJoint != m_cJointEquippedEntity.GetAllJoints().end();
          ++itJoint) {
         
         CDynamics3DJoint* pcJoint = *std::find(m_vecLocalJoints.begin(),
                                                m_vecLocalJoints.end(),
                                                (*itJoint)->GetId());
         /* Write the motor target velocities to the joints */
         /* add method for enable/disable? */
         //@todo for correctness it might be necerry to negate the z/y velocities according to the transforms

         pcJoint->SetActuatorTargetVelocity(CDynamics3DJoint::LINEAR_X,
                                            (*itJoint)->GetAxisLinearX().m_fActuatorTargetVelocity);
         pcJoint->SetActuatorTargetVelocity(CDynamics3DJoint::LINEAR_Y,
                                            (*itJoint)->GetAxisLinearZ().m_fActuatorTargetVelocity);
         pcJoint->SetActuatorTargetVelocity(CDynamics3DJoint::LINEAR_Z,
                                            (*itJoint)->GetAxisLinearY().m_fActuatorTargetVelocity);

         pcJoint->SetActuatorTargetVelocity(CDynamics3DJoint::ANGULAR_X,
                                            (*itJoint)->GetAxisAngularX().m_fActuatorTargetVelocity);
         pcJoint->SetActuatorTargetVelocity(CDynamics3DJoint::ANGULAR_Y,
                                            (*itJoint)->GetAxisAngularZ().m_fActuatorTargetVelocity);
         pcJoint->SetActuatorTargetVelocity(CDynamics3DJoint::ANGULAR_Z,
                                            (*itJoint)->GetAxisAngularY().m_fActuatorTargetVelocity);
      }
      // @todo run this method on the reference body!
      //->ActivateRigidBody();
      // Add method get reference body which sets the pointer
   }

   /****************************************/
   /****************************************/

   btTransform CDynamics3DRobotModel::GetModelCoordinates() const {
      const std::string& strReferenceBodyId = 
         m_cRobotEntity.GetBodyEquippedEntity().GetReferenceBody().GetId();
      
      const CDynamics3DBody& sReferenceBodyConfiguration = **std::find(m_vecLocalBodies.begin(),
                                                                       m_vecLocalBodies.end(),
                                                                       strReferenceBodyId);

      return (sReferenceBodyConfiguration.GetMotionStateTransform() *
              sReferenceBodyConfiguration.GetPositionalOffset().inverse());
   }

   /****************************************/
   /****************************************/

   btBoxShape* CDynamics3DRobotModel::CBoxShapeManager::RequestBoxShape(const btVector3& c_half_extents) {
      std::vector<CResource>::iterator itResource;      
      for(itResource = m_vecResources.begin();
          itResource != m_vecResources.end();
          ++itResource) {
         if(itResource->m_cHalfExtents == c_half_extents) break;
      }      
      // if it doesn't exist, create a new one
      if(itResource == m_vecResources.end()) {
         itResource = m_vecResources.insert(itResource, 
                                            CResource(c_half_extents, new btBoxShape(c_half_extents)));
      }
      itResource->m_unInUseCount++;
      return itResource->m_cShape;
   }

   /****************************************/
   /****************************************/

   void CDynamics3DRobotModel::CBoxShapeManager::ReleaseBoxShape(const btBoxShape* pc_release) {
      std::vector<CResource>::iterator itResource;      
      for(itResource = m_vecResources.begin();
          itResource != m_vecResources.end();
          ++itResource) {
         if(itResource->m_cShape == pc_release) break;
      }
      // if it doesn't exist, throw an exception
      if(itResource == m_vecResources.end()) {
         THROW_ARGOSEXCEPTION("Attempt to release unknown btBoxShape from the box shape manager!");
      }
      itResource->m_unInUseCount--;
      if(itResource->m_unInUseCount == 0) {
         delete itResource->m_cShape;
         m_vecResources.erase(itResource);
      }
   }

   /****************************************/
   /****************************************/

   CDynamics3DRobotModel::CBoxShapeManager::CResource::CResource(const btVector3& c_half_extents,
                                                                 btBoxShape* c_shape) : 
      m_cHalfExtents(c_half_extents),
      m_cShape(c_shape),
      m_unInUseCount(0) {}

   /****************************************/
   /****************************************/

   btCylinderShape* CDynamics3DRobotModel::CCylinderShapeManager::RequestCylinderShape(const btVector3& c_half_extents) {
      std::vector<CResource>::iterator itResource;      
      for(itResource = m_vecResources.begin();
          itResource != m_vecResources.end();
          ++itResource) {
         if(itResource->m_cHalfExtents == c_half_extents) break;
      }      
      // if it doesn't exist, create a new one
      if(itResource == m_vecResources.end()) {
         itResource = m_vecResources.insert(itResource, 
                                            CResource(c_half_extents, new btCylinderShape(c_half_extents)));
      }
      itResource->m_unInUseCount++;
      return itResource->m_cShape;
   }

   /****************************************/
   /****************************************/

   void CDynamics3DRobotModel::CCylinderShapeManager::ReleaseCylinderShape(const btCylinderShape* pc_release) {
      std::vector<CResource>::iterator itResource;      
      for(itResource = m_vecResources.begin();
          itResource != m_vecResources.end();
          ++itResource) {
         if(itResource->m_cShape == pc_release) break;
      }
      // if it doesn't exist, throw an exception
      if(itResource == m_vecResources.end()) {
         THROW_ARGOSEXCEPTION("Attempt to release unknown btCylinderShape from the cylinder shape manager!");
      }
      itResource->m_unInUseCount--;
      if(itResource->m_unInUseCount == 0) {
         delete itResource->m_cShape;
         m_vecResources.erase(itResource);
      }
   }

   /****************************************/
   /****************************************/

   CDynamics3DRobotModel::CCylinderShapeManager::CResource::CResource(const btVector3& c_half_extents,
                                                                      btCylinderShape* c_shape) : 
      m_cHalfExtents(c_half_extents),
      m_cShape(c_shape),
      m_unInUseCount(0) {}

   /****************************************/
   /****************************************/

   btSphereShape* CDynamics3DRobotModel::CSphereShapeManager::RequestSphereShape(Real f_radius) {
      std::vector<CResource>::iterator itResource;      
      for(itResource = m_vecResources.begin();
          itResource != m_vecResources.end();
          ++itResource) {
         if(itResource->m_fRadius == f_radius) break;
      }      
      // if it doesn't exist, create a new one
      if(itResource == m_vecResources.end()) {
         itResource = m_vecResources.insert(itResource, 
                                            CResource(f_radius, new btSphereShape(f_radius)));
      }
      itResource->m_unInUseCount++;
      return itResource->m_cShape;
   }

   /****************************************/
   /****************************************/

   void CDynamics3DRobotModel::CSphereShapeManager::ReleaseSphereShape(const btSphereShape* pc_release) {
      std::vector<CResource>::iterator itResource;      
      for(itResource = m_vecResources.begin();
          itResource != m_vecResources.end();
          ++itResource) {
         if(itResource->m_cShape == pc_release) break;
      }
      // if it doesn't exist, throw an exception
      if(itResource == m_vecResources.end()) {
         THROW_ARGOSEXCEPTION("Attempt to release unknown btSphereShape from the sphere shape manager!");
      }
      itResource->m_unInUseCount--;
      if(itResource->m_unInUseCount == 0) {
         delete itResource->m_cShape;
         m_vecResources.erase(itResource);
      }
   }

   /****************************************/
   /****************************************/

   CDynamics3DRobotModel::CSphereShapeManager::CResource::CResource(Real f_radius,
                                                                    btSphereShape* c_shape) : 
      m_fRadius(f_radius),
      m_cShape(c_shape),
      m_unInUseCount(0) {}

   /****************************************/
   /****************************************/

   CDynamics3DRobotModel::CBoxShapeManager      CDynamics3DRobotModel::m_cBoxShapeManager;
   CDynamics3DRobotModel::CCylinderShapeManager CDynamics3DRobotModel::m_cCylinderShapeManager;
   CDynamics3DRobotModel::CSphereShapeManager   CDynamics3DRobotModel::m_cSphereShapeManager;

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_DYNAMICS3D_OPERATIONS_ON_ENTITY(CRobotEntity, CDynamics3DRobotModel);

   /****************************************/
   /****************************************/

}

