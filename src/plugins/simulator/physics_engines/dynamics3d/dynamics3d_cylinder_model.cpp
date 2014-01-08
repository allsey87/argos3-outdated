/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_cylinder_model.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "dynamics3d_cylinder_model.h"

#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_engine.h>

namespace argos {

   /****************************************/
   /****************************************/

   CDynamics3DCylinderModel::CDynamics3DCylinderModel(CDynamics3DEngine& c_engine,
                                                      CCylinderEntity& c_cylinder) :
      CDynamics3DModel(c_engine, c_cylinder.GetEmbodiedEntity(), c_cylinder.GetId()),
      m_cCylinderEntity(c_cylinder) {
      /* When defining size of objects we must manually swap the Z and Y components */

      m_pcCylinderCollisionShape = m_cCylinderShapeManager.RequestCylinderShape(btVector3(c_cylinder.GetRadius(),
                                                                                          c_cylinder.GetHeight() * 0.5f,
                                                                                          c_cylinder.GetRadius()));
      btTransform cCylinderGeometricOffset(
         btQuaternion(0.0f, 0.0f, 0.0f, 1.0f), 
         btVector3(0.0f, -c_cylinder.GetHeight() * 0.5f, 0.0f));
      Real fMass = c_cylinder.GetEmbodiedEntity().IsMovable() ? c_cylinder.GetMass() : 0.0f;
      m_vecLocalBodies.push_back(new CDynamics3DBody(this,
                                                     "cylinder",
                                                     m_pcCylinderCollisionShape,
                                                     btTransform::getIdentity(),
                                                     cCylinderGeometricOffset,
                                                     fMass));

      /* move the model to the specified coordinates */
      const CQuaternion& cAQuat = GetEmbodiedEntity().GetInitOrientation();
      const CVector3& cAVec = GetEmbodiedEntity().GetInitPosition();

      btVector3 cBtVec = ARGoSToBullet(cAVec);
      btQuaternion cBtQuat = ARGoSToBullet(cAQuat);

      SetModelCoordinates(btTransform(cBtQuat, cBtVec));
   }
   
   /****************************************/
   /****************************************/
   
   CDynamics3DCylinderModel::~CDynamics3DCylinderModel() {
      m_cCylinderShapeManager.ReleaseCylinderShape(m_pcCylinderCollisionShape);
    }
   
   /****************************************/
   /****************************************/

   void CDynamics3DCylinderModel::UpdateEntityStatus() {
      if(m_cCylinderEntity.GetEmbodiedEntity().IsMovable()) {
         const btTransform& cUpdateTransform = GetModelCoordinates();         
         GetEmbodiedEntity().SetPosition(BulletToARGoS(cUpdateTransform.getOrigin()));
         GetEmbodiedEntity().SetOrientation(BulletToARGoS(cUpdateTransform.getRotation()));
         m_cCylinderEntity.UpdateComponents();
      }
   }

   /****************************************/
   /****************************************/

   btTransform CDynamics3DCylinderModel::GetModelCoordinates() const {
      return m_vecLocalBodies[0]->GetMotionStateTransform();
   }

   /****************************************/
   /****************************************/

   btCylinderShape* CDynamics3DCylinderModel::CCylinderShapeManager::RequestCylinderShape(const btVector3& c_half_extents) {
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

   void CDynamics3DCylinderModel::CCylinderShapeManager::ReleaseCylinderShape(const btCylinderShape* pc_release) {
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

   CDynamics3DCylinderModel::CCylinderShapeManager::CResource::CResource(const btVector3& c_half_extents,
                                                               btCylinderShape* c_shape) : 
      m_cHalfExtents(c_half_extents),
      m_cShape(c_shape),
      m_unInUseCount(0) {}

   /****************************************/
   /****************************************/

   CDynamics3DCylinderModel::CCylinderShapeManager CDynamics3DCylinderModel::m_cCylinderShapeManager;

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_DYNAMICS3D_OPERATIONS_ON_ENTITY(CCylinderEntity, CDynamics3DCylinderModel);

}
