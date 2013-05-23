/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics3d/sphere/dynamics3d_sphere_model.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef DYNAMICS3D_SPHERE_MODEL_H
#define DYNAMICS3D_SPHERE_MODEL_H

namespace argos {
   class CDynamics3DEngine;
   class CDynamics3DSphereModel;
}

#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_model.h>
#include <argos3/plugins/simulator/physics_engines/dynamics3d/sphere/sphere_entity.h>

namespace argos {

   class CDynamics3DSphereModel : public CDynamics3DModel {

   public:
      
      CDynamics3DSphereModel(CDynamics3DEngine& c_engine,
                             CSphereEntity& c_entity);
      virtual ~CDynamics3DSphereModel();
      virtual void UpdateEntityStatus();
      virtual void UpdateFromEntityStatus() {}
      
   protected:

      virtual btTransform GetModelCoordinates() const;
      
   private:

      CSphereEntity&           m_cSphereEntity;
      btSphereShape*           m_pcSphereCollisionShape;

      class CSphereShapeManager {
         public:
            btSphereShape* RequestSphereShape(Real f_radius);
            void ReleaseSphereShape(const btSphereShape* pc_release);
         private:
            struct CResource {
               CResource(Real f_radius, btSphereShape* c_shape);
               Real m_fRadius;
               btSphereShape* m_cShape;
               UInt32 m_unInUseCount;
            };
            std::vector<CResource> m_vecResources;
      };

      static CSphereShapeManager m_cSphereShapeManager;
   };
}

#endif
