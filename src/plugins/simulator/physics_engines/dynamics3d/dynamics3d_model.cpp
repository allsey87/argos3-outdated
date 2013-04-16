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

   bool CDynamics3DModel::MoveTo(const CVector3& c_position,
                                 const CQuaternion& c_orientation,
                                 bool b_check_only) {

      //
      const std::string& strId = GetEmbodiedEntity().GetParent().GetId();

fprintf(stderr, "MoveTo called on %s!\n", strId.c_str());

 fprintf(stderr, "Requested coordinates are: [%.3f, %.3f, %.3f]", c_position.GetX(), c_position.GetY(), c_position.GetZ());


      const btTransform& cCurrentCoordinates = GetModelCoordinates();
      const btTransform& cMoveToCoordinates = btTransform(ARGoSToBullet(c_orientation),
                                                          ARGoSToBullet(c_position));           
      
      SetModelCoordinates(cMoveToCoordinates);

      bool bModelHasCollision = m_cEngine.IsModelCollidingWithSomething(*this);    
      
      // Check if we are performing the move operation or not
      if(bModelHasCollision == true || b_check_only == true) {
         fprintf(stderr, "location occupied or check - restoring entity location!\n");
         SetModelCoordinates(cCurrentCoordinates);
      }
      else {
         fprintf(stderr, "alles klar!\n");
      }

      // return whether the MoveTo was or would have been sucessful
      return !bModelHasCollision;
   }


   void CDynamics3DModel::Reset() {
      // reset each body and clearing all velocities and forces
      for(std::map<std::string, SBodyConfiguration>::iterator itBodyConfiguration = m_mapLocalBodyConfigurations.begin();
          itBodyConfiguration !=  m_mapLocalBodyConfigurations.end();
          ++itBodyConfiguration) {
         
         btMotionState* pcMotionState = itBodyConfiguration->second.m_pcMotionState;
         pcMotionState->reset();
         itBodyConfiguration->second.m_pcRigidBody->setMotionState(pcMotionState);
            
         itBodyConfiguration->second.m_pcRigidBody->setAngularVelocity(btVector3(0.0f, 0.0f, 0.0f));
         itBodyConfiguration->second.m_pcRigidBody->setLinearVelocity(btVector3(0.0f, 0.0f, 0.0f));
         itBodyConfiguration->second.m_pcRigidBody->clearForces();
      }
   }
   

   void CDynamics3DModel::CalculateBoundingBox() {
      btVector3 cAabbMin, cAabbMax, cBodyAabbMin, cBodyAabbMax;
      bool bAabbVectorInitRequired = true;

      for(std::map<std::string, SBodyConfiguration>::const_iterator itBodyConfiguration = m_mapLocalBodyConfigurations.begin();
          itBodyConfiguration != m_mapLocalBodyConfigurations.end();
          itBodyConfiguration++) {
            
         // get the axis aligned bounding box for the current body
         itBodyConfiguration->second.m_pcCollisionShape->getAabb(itBodyConfiguration->second.m_pcRigidBody->getWorldTransform(), cBodyAabbMin, cBodyAabbMax);
            
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

   bool CDynamics3DModel::IsCollidingWithSomething() const {
      return m_cEngine.IsModelCollidingWithSomething(*this);
   }

   bool CDynamics3DModel::CheckIntersectionWithRay(Real& f_t_on_ray, const CRay3& c_ray) const {
      btTransform cRayStartTransform(btQuaternion::getIdentity(), ARGoSToBullet(c_ray.GetStart()));
      btTransform cRayEndTransform(btQuaternion::getIdentity(), ARGoSToBullet(c_ray.GetEnd()));
      btCollisionObject cTempCollisionObject;         
      bool bIntersectionOccured = false;         
      Real fModelIntersectDist;
      Real fBodyIntersectDist; 
         
      //@todo use SbodyComponent backed via a std::vector to increase speed (reduced cache misses)
      for(std::map<std::string, SBodyConfiguration>::const_iterator itBodyConfiguration = m_mapLocalBodyConfigurations.begin();
          itBodyConfiguration != m_mapLocalBodyConfigurations.end();
          itBodyConfiguration++) {

         // This object cannot be used twice or reinitialised, we must construct it on every iteration
         btCollisionWorld::ClosestRayResultCallback cResult(cRayStartTransform.getOrigin(),
                                                            cRayEndTransform.getOrigin());

         // test for a collision against the current body
         btCollisionWorld::rayTestSingle(cRayStartTransform,
                                         cRayEndTransform,
                                         &cTempCollisionObject,
                                         itBodyConfiguration->second.m_pcCollisionShape,
                                         itBodyConfiguration->second.m_pcRigidBody->getWorldTransform(),
                                         cResult);

         // if this body intersected the ray, we compute whether or not this has been the closest intersection
         if (cResult.hasHit()) {
            fBodyIntersectDist = (cResult.m_hitPointWorld - cRayStartTransform.getOrigin()).length();
            if(bIntersectionOccured == false) {
               fModelIntersectDist = fBodyIntersectDist;
               bIntersectionOccured = true;
            }
            else {
               fModelIntersectDist = (fModelIntersectDist > fBodyIntersectDist) ? fBodyIntersectDist : fModelIntersectDist;
            }
         }
      }
      f_t_on_ray = fModelIntersectDist / c_ray.GetLength();
      return bIntersectionOccured;
   }


   void CDynamics3DModel::SetModelCoordinates(const btTransform& c_coordinates) {

      // Calculate the position and orientation of the entity using the reference body
      const btTransform& cCurrentCoordinates = GetModelCoordinates();

      // Iterate across each body in the model
      std::map<std::string, SBodyConfiguration>::const_iterator itBodyConfiguration;

      for(itBodyConfiguration = m_mapLocalBodyConfigurations.begin();
          itBodyConfiguration != m_mapLocalBodyConfigurations.end();
          ++itBodyConfiguration) {

         btDefaultMotionState* pcMotionState = itBodyConfiguration->second.m_pcMotionState;
         btRigidBody* pcRigidBody = itBodyConfiguration->second.m_pcRigidBody;

         // Calculate the transform between the entity and the body
         btTransform cOffsetTransform = cCurrentCoordinates.inverse() *
            pcMotionState->m_graphicsWorldTrans;
         
         // Apply this transform to the new entity location to find the location of the body
         pcMotionState->m_graphicsWorldTrans = c_coordinates * cOffsetTransform;

         // Tell Bullet to update body by resetting the motion state
         pcRigidBody->setMotionState(pcMotionState);

         // activate the body
         pcRigidBody->activate();
      }
   }


}
