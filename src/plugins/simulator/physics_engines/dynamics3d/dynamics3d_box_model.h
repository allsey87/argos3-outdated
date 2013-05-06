/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_box_model.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef DYNAMICS3D_BOX_MODEL_H
#define DYNAMICS3D_BOX_MODEL_H

namespace argos {
   class CDynamics3DEngine;
   class CDynamics3DBoxModel;
}

#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_model.h>
#include <argos3/plugins/simulator/entities/box_entity.h>


namespace argos {

   class CDynamics3DBoxModel : public CDynamics3DModel {

   public:
      
      CDynamics3DBoxModel(CDynamics3DEngine& c_engine,
                          CBoxEntity& c_box);
      virtual ~CDynamics3DBoxModel();

      virtual void UpdateEntityStatus();
      virtual void UpdateFromEntityStatus() {}
 
   protected:
      
      virtual btTransform GetModelCoordinates() const;
      
   private:

      CBoxEntity&                m_cBoxEntity;
      btBoxShape*                m_pcBoxCollisionShape;

      class CBoxShapeManager {
         public:
            btBoxShape* RequestBoxShape(const btVector3& c_half_extents);
            void ReleaseBoxShape(const btBoxShape* pc_release);
         private:
            struct CResource {
               CResource(const btVector3& c_half_extents, btBoxShape* c_shape);
               btVector3 m_cHalfExtents;
               btBoxShape* m_cShape;
               UInt32 m_unInUseCount;
            };
            std::vector<CResource> m_vecResources;
      };

      static CBoxShapeManager m_cBoxShapeManager;
   };
}

#endif
