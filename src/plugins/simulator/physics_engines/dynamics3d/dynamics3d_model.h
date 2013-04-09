/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_model.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef DYNAMICS3D_MODEL_H
#define DYNAMICS3D_MODEL_H

namespace argos {
   class  CDynamics3DEngine;
}

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
   
   inline CVector3 BulletToARGoS(const btVector3& c_bt_vector) {
      return CVector3(c_bt_vector.getX(), -c_bt_vector.getZ(), c_bt_vector.getY());
   }
   
   inline btVector3 ARGoSToBullet(const CVector3& c_a_vector) {
      return btVector3(c_a_vector.GetX(), c_a_vector.GetZ(), -c_a_vector.GetY());
   }
   
   inline CQuaternion BulletToARGoS(const btQuaternion& c_bt_quaternion) {
      return CQuaternion(c_bt_quaternion.getW(), c_bt_quaternion.getX(),
                         -c_bt_quaternion.getZ(), c_bt_quaternion.getY());
   }
   
   inline btQuaternion ARGoSToBullet(const CQuaternion& c_a_quaternion) {
      return btQuaternion(c_a_quaternion.GetX(), c_a_quaternion.GetZ(), 
                          -c_a_quaternion.GetY(), c_a_quaternion.GetW());
   }
   
   /****************************************/
   /****************************************/

   class CDynamics3DModel : public CPhysicsModel {

   public:
      
      typedef std::vector<CDynamics3DModel*> TVector;
      typedef std::map<std::string, CDynamics3DModel*> TMap;
      typedef std::tr1::unordered_multimap<std::string, CDynamics3DModel*> TMultiMap;

   public:

      CDynamics3DModel(CDynamics3DEngine& c_engine,
                       CEmbodiedEntity& c_entity) :
         CPhysicsModel(c_engine, c_entity),
         m_cEngine(c_engine) {}
      virtual ~CDynamics3DModel() {}

      virtual bool MoveTo(const CVector3& c_position,
                          const CQuaternion& c_orientation,
                          bool b_check_only = false) {
         return false;                   
      }

      virtual void Reset() {
         // reset each body and clearing all velocities and forces
         for(std::map<std::string, btRigidBody*>::iterator itBody = m_mapLocalRigidBodies.begin();
             itBody !=  m_mapLocalRigidBodies.end();
             itBody++) {
         
            btMotionState* pcMotionState = itBody->second->getMotionState();
            pcMotionState->reset();
            itBody->second->setMotionState(pcMotionState);
            
            itBody->second->setAngularVelocity(btVector3(0.0f, 0.0f, 0.0f));
            itBody->second->setLinearVelocity(btVector3(0.0f, 0.0f, 0.0f));
            itBody->second->clearForces();
         }
      }

      virtual const btTransform& GetModelWorldTransform() const = 0;
      
      virtual void UpdateEntityStatus() = 0;
      virtual void UpdateFromEntityStatus() = 0;
      
      inline const std::map<std::string, btRigidBody*>& GetRigidBodies() const {
         return m_mapLocalRigidBodies;
      }
      
      inline const std::map<std::string, btTypedConstraint*>& GetConstraints() const {
         return m_mapLocalConstraints;
      }

      virtual void CalculateBoundingBox() {
         btVector3 cAabbMin, cAabbMax, cBodyAabbMin, cBodyAabbMax;
         bool bAabbVectorInitRequired = true;

         //@todo integrate the new SBodyConfiguration structure
         for(std::map<std::string, btRigidBody*>::const_iterator itBody = m_mapLocalRigidBodies.begin();
             itBody != m_mapLocalRigidBodies.end();
             itBody++) {
            
            itBody->second->getCollisionShape()->getAabb(itBody->second->getWorldTransform(), cBodyAabbMin, cBodyAabbMax);
            
            if(bAabbVectorInitRequired == true) {
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
         GetBoundingBox().MinCorner = BulletToARGoS(cAabbMin);
         GetBoundingBox().MaxCorner = BulletToARGoS(cAabbMax);
      }

      virtual bool IsCollidingWithSomething() const {
         return false;
      }

      virtual bool CheckIntersectionWithRay(Real& f_t_on_ray, const CRay3& c_ray) const {
         
fprintf(stderr, "CheckIntersectionWithRay called on entity %s\n", m_cEmbodiedEntity.GetParent().GetId().c_str());

         btVector3 cRayStart = ARGoSToBullet(c_ray.GetStart());
         btVector3 cRayEnd = ARGoSToBullet(c_ray.GetEnd());
      
         btTransform cRayFromTransform(btTransform::getIdentity());
	 btTransform cRayToTransform(btTransform::getIdentity());
         
         cRayFromTransform.setOrigin(cRayStart);
         cRayToTransform.setOrigin(cRayEnd);
         
         btCollisionWorld::ClosestRayResultCallback cResult(cRayStart, cRayEnd);
         
         btCollisionObject cTempCollisionObject;
         
         btCollisionWorld::rayTestSingle(cRayFromTransform,
                                         cRayToTransform,
                                         &cTempCollisionObject,
                                         &m_cModelCompositeShape,
                                         GetModelWorldTransform(),
                                         cResult);
         
         if (cResult.hasHit()) {
            const btVector3& cHitPoint = cResult.m_hitPointWorld;
            f_t_on_ray = (cHitPoint - cRayStart).length() / c_ray.GetLength();
            fprintf(stderr, "Intersection detected! Hit point occured at %.2f\n", f_t_on_ray);
            
            return true;
         }
         else {
            return false;
         }
      }

      virtual void UpdateModelCompositeShape() {
         // remove the old shapes in the model
         for(size_t i = 0; i < size_t(m_cModelCompositeShape.getNumChildShapes()); ++i) {
            m_cModelCompositeShape.removeChildShapeByIndex(i);
         }

         const btTransform& cInverseModelWorldTransform = GetModelWorldTransform().inverse();
         
         for(std::map<std::string, btRigidBody*>::const_iterator it = m_mapLocalRigidBodies.begin();
             it != m_mapLocalRigidBodies.end();
             it++) {

            m_cModelCompositeShape.addChildShape(cInverseModelWorldTransform * it->second->getWorldTransform(),
                                                 it->second->getCollisionShape());
         }

         //fprintf(stderr, "CDynamics3DModel::UpdateCompositeShape() called!\n");
      }

      
   private:
      
      btCompoundShape m_cModelCompositeShape;

   protected:
      CDynamics3DEngine&      m_cEngine;
      
      std::map<std::string, btRigidBody*> m_mapLocalRigidBodies;
      std::map<std::string, btTypedConstraint*> m_mapLocalConstraints;

   };

}

#endif
