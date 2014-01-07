/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_model.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "dynamics3d_model.h"

#include <argos3/core/simulator/physics_engine/physics_model.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_engine.h>

#include <tr1/unordered_map>

//remove this include - added for debugging
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <cstdio>

namespace argos {

   /****************************************/
   /****************************************/

   CDynamics3DModel::~CDynamics3DModel() {
       for(CDynamics3DJoint::TVector::iterator itJoint = m_vecLocalJoints.begin();
          itJoint != m_vecLocalJoints.end();
          ++itJoint) {
         delete *itJoint;
      }
      for(CDynamics3DBody::TVector::iterator itBody = m_vecLocalBodies.begin();
          itBody != m_vecLocalBodies.end();
          ++itBody) {
         delete *itBody;
      }
   }

   /****************************************/
   /****************************************/


   bool CDynamics3DModel::MoveTo(const CVector3& c_position,
                                 const CQuaternion& c_orientation,
                                 bool b_check_only) {

      //const btTransform& cCurrentCoordinates = GetModelCoordinates();
      const btTransform& cMoveToCoordinates  = btTransform(ARGoSToBullet(c_orientation),
                                                           ARGoSToBullet(c_position));           
      
      SetModelCoordinates(cMoveToCoordinates);

      bool bModelHasCollision = m_cEngine.IsModelCollidingWithSomething(*this);    

      // If this object wasn't a check, resync the model with the entity
      if(bModelHasCollision == false || b_check_only == false) {
         UpdateEntityStatus();
      }

      // return whether the MoveTo was or would have been sucessful
      return !bModelHasCollision;
   }

   /****************************************/
   /****************************************/

   void CDynamics3DModel::Reset() {
      for(CDynamics3DBody::TVector::iterator itBody = m_vecLocalBodies.begin();
          itBody != m_vecLocalBodies.end();
          ++itBody) {
         (*itBody)->Reset();
      }

      for(CDynamics3DJoint::TVector::iterator itJoint = m_vecLocalJoints.begin();
          itJoint != m_vecLocalJoints.end();
          ++itJoint) {
         (*itJoint)->Reset();
      }

      btTransform cModelResetTransform(ARGoSToBullet(GetEmbodiedEntity().GetInitOrientation()),
                                       ARGoSToBullet(GetEmbodiedEntity().GetInitPosition()));

      SetModelCoordinates(cModelResetTransform);
      UpdateEntityStatus();
   }

   /****************************************/
   /****************************************/

   void CDynamics3DModel::CalculateBoundingBox() {
      btVector3 cAabbMin, cAabbMax, cBodyAabbMin, cBodyAabbMax;
      bool bAabbVectorInitRequired = true;

      for(CDynamics3DBody::TVector::iterator itBody = m_vecLocalBodies.begin();
          itBody != m_vecLocalBodies.end();
          itBody++) {
            
         // get the axis aligned bounding box for the current body
         (*itBody)->GetCollisionShape().getAabb((*itBody)->GetRigidBodyTransform(),
                                                cBodyAabbMin,
                                                cBodyAabbMax);
            
         if(bAabbVectorInitRequired == true) {
            // this is the first body in the model, use it's axis aligned bounding box.
            cAabbMin = cBodyAabbMin;
            cAabbMax = cBodyAabbMax;
            bAabbVectorInitRequired = false;
         }
         else {
            // compute the maximum and minimum corners
            cAabbMin.setX(cAabbMin.getX() < cBodyAabbMin.getX() ? cAabbMin.getX() : cBodyAabbMin.getX());
            cAabbMin.setY(cAabbMin.getY() < cBodyAabbMin.getY() ? cAabbMin.getY() : cBodyAabbMin.getY());
            cAabbMin.setZ(cAabbMin.getZ() < cBodyAabbMin.getZ() ? cAabbMin.getZ() : cBodyAabbMin.getZ());

            cAabbMax.setX(cAabbMax.getX() > cBodyAabbMax.getX() ? cAabbMax.getX() : cBodyAabbMax.getX());
            cAabbMax.setY(cAabbMax.getY() > cBodyAabbMax.getY() ? cAabbMax.getY() : cBodyAabbMax.getY());
            cAabbMax.setZ(cAabbMax.getZ() > cBodyAabbMax.getZ() ? cAabbMax.getZ() : cBodyAabbMax.getZ());
         }
      }

      /* When writing the bounding box back to ARGoS we must manually swap the Y component, to compensate for
         the different coordinate systems between the two worlds, which effectively rotates the bounding box */
      GetBoundingBox().MinCorner = BulletToARGoS(cAabbMin);
      GetBoundingBox().MinCorner.SetY(BulletToARGoS(cAabbMax).GetY());
      GetBoundingBox().MaxCorner = BulletToARGoS(cAabbMax);
      GetBoundingBox().MaxCorner.SetY(BulletToARGoS(cAabbMin).GetY());
   }

   /****************************************/
   /****************************************/

   bool CDynamics3DModel::IsCollidingWithSomething() const {
      return m_cEngine.IsModelCollidingWithSomething(*this);
   }

   /****************************************/
   /****************************************/

   void CDynamics3DModel::SetModelCoordinates(const btTransform& c_coordinates) {

      // Calculate the position and orientation of the entity using the reference body
      const btTransform& cCurrentCoordinates = GetModelCoordinates();

      // Iterate across each body in the model
      for(CDynamics3DBody::TVector::iterator itBody = m_vecLocalBodies.begin();
          itBody != m_vecLocalBodies.end();
          ++itBody) {

         // Calculate the transform between the entity and the body
         const btTransform& cOffsetTransform = cCurrentCoordinates.inverse() *
            (*itBody)->GetMotionStateTransform();
         
         // Apply this transform to the new entity location to find the location of the body
         (*itBody)->SetMotionStateTransform(c_coordinates * cOffsetTransform);

         // Tell Bullet to update body by resetting the motion state
         (*itBody)->SynchronizeMotionState();

         // activate the body
         (*itBody)->ActivateRigidBody();

         // Update the bounding box
         CalculateBoundingBox();
      }
   }

   /****************************************/
   /****************************************/

   bool operator==(const CDynamics3DModel* pc_dyn3d_model, const std::string& str_id) {
      return (pc_dyn3d_model->GetId() == str_id);
   }

   /****************************************/
   /****************************************/

}
