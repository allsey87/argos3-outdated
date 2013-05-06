/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_box_model.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef DYNAMICS3D_CYLINDER_MODEL_H
#define DYNAMICS3D_CYLINDER_MODEL_H

namespace argos {
   class CDynamics3DEngine;
   class CDynamics3DCylinderModel;
}

#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_model.h>
#include <argos3/plugins/simulator/entities/cylinder_entity.h>


namespace argos {

   class CDynamics3DCylinderModel : public CDynamics3DModel {

   public:
      
      CDynamics3DCylinderModel(CDynamics3DEngine& c_engine,
                               CCylinderEntity& c_entity);
      virtual ~CDynamics3DCylinderModel();
      virtual void UpdateEntityStatus();
      virtual void UpdateFromEntityStatus() {}
      
   protected:

      virtual btTransform GetModelCoordinates() const;
      
   private:

      CCylinderEntity&           m_cCylinderEntity;
      btCylinderShape*           m_pcCylinderCollisionShape;

      class CCylinderShapeManager {
         public:
            btCylinderShape* RequestCylinderShape(const btVector3& c_half_extents);
            void ReleaseCylinderShape(const btCylinderShape* pc_release);
         private:
            struct CResource {
               CResource(const btVector3& c_half_extents, btCylinderShape* c_shape);
               btVector3 m_cHalfExtents;
               btCylinderShape* m_cShape;
               UInt32 m_unInUseCount;
            };
            std::vector<CResource> m_vecResources;
      };

      static CCylinderShapeManager m_cCylinderShapeManager;
   };
}

#endif
