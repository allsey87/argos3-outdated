/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics3d/sphere/dynamics3d_sphere_model.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "dynamics3d_sphere_model.h"

#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_engine.h>

namespace argos {

   /****************************************/
   /****************************************/

   CDynamics3DSphereModel::CDynamics3DSphereModel(CDynamics3DEngine& c_engine,
                                                  CSphereEntity& c_sphere) :
      CDynamics3DModel(c_engine, c_sphere.GetEmbodiedEntity(), c_sphere.GetId()),
      m_cSphereEntity(c_sphere) {
      /* When defining size of objects we must manually swap the Z and Y components */
      m_pcSphereCollisionShape = m_cSphereShapeManager.RequestSphereShape(c_sphere.GetRadius());

      btTransform cSphereGeometricOffset(
         btQuaternion(0.0f, 0.0f, 0.0f, 1.0f), 
         btVector3(0.0f, -c_sphere.GetRadius(), 0.0f));
      Real fMass = c_sphere.GetEmbodiedEntity().IsMovable() ? c_sphere.GetMass() : 0.0f;
      m_vecLocalBodies.push_back(new CDynamics3DBody(this,
                                                     "Sphere",
                                                     m_pcSphereCollisionShape,
                                                     btTransform::getIdentity(),
                                                     cSphereGeometricOffset,
                                                     fMass));
      /* move the model to the specified coordinates */
      SetModelCoordinates(btTransform(ARGoSToBullet(GetEmbodiedEntity().GetInitOrientation()),
                                      ARGoSToBullet(GetEmbodiedEntity().GetInitPosition())));
   }
   
   /****************************************/
   /****************************************/
   
   CDynamics3DSphereModel::~CDynamics3DSphereModel() {
      m_cSphereShapeManager.ReleaseSphereShape(m_pcSphereCollisionShape);
   }
   
   /****************************************/
   /****************************************/

   void CDynamics3DSphereModel::UpdateEntityStatus() {
      if(m_cSphereEntity.GetEmbodiedEntity().IsMovable()) {
         const btTransform& cUpdateTransform = GetModelCoordinates();         
         GetEmbodiedEntity().SetPosition(BulletToARGoS(cUpdateTransform.getOrigin()));
         GetEmbodiedEntity().SetOrientation(BulletToARGoS(cUpdateTransform.getRotation()));
         m_cSphereEntity.UpdateComponents();
      }
   }

   /****************************************/
   /****************************************/

   btTransform CDynamics3DSphereModel::GetModelCoordinates() const {
      return m_vecLocalBodies[0]->GetMotionStateTransform();
   }

   /****************************************/
   /****************************************/

   btSphereShape* CDynamics3DSphereModel::CSphereShapeManager::RequestSphereShape(Real f_radius) {
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

   void CDynamics3DSphereModel::CSphereShapeManager::ReleaseSphereShape(const btSphereShape* pc_release) {
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

   CDynamics3DSphereModel::CSphereShapeManager::CResource::CResource(Real f_radius,
                                                                     btSphereShape* c_shape) : 
      m_fRadius(f_radius),
      m_cShape(c_shape),
      m_unInUseCount(0) {}

   /****************************************/
   /****************************************/

   CDynamics3DSphereModel::CSphereShapeManager CDynamics3DSphereModel::m_cSphereShapeManager;

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_DYNAMICS3D_OPERATIONS_ON_ENTITY(CSphereEntity, CDynamics3DSphereModel);

}
