/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_heightfield_model.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef DYNAMICS3D_HEIGHTFIELD_MODEL_H
#define DYNAMICS3D_HEIGHTFIELD_MODEL_H

namespace argos {
   class CDynamics3DEngine;
   class CDynamics3DHeightFieldModel;
}

#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_model.h>
#include <argos3/plugins/simulator/entities/terrain_entity.h>


namespace argos {

   class CDynamics3DHeightFieldModel : public CDynamics3DModel {

   public:
      
      CDynamics3DHeightFieldModel(CDynamics3DEngine& c_engine,
                                  CHeightFieldEntity& c_Terrain);
      virtual ~CDynamics3DHeightFieldModel();
      
      virtual bool CheckIntersectionWithRay(Real& f_t_on_ray,
                                            const CRay3& c_ray) const;

      virtual bool MoveTo(const CVector3& c_position,
                          const CQuaternion& c_orientation,
                          bool b_check_only = false) {
         return false; // @todo support moving terrain to produce random maps
      }

      virtual void Reset();

      virtual void UpdateEntityStatus() {}
      virtual void UpdateFromEntityStatus() {}
      
      virtual std::vector<btRigidBody*>& GetRigidBodies() {
         return m_vecRigidBodies;
      }
      

   private:

      CHeightFieldEntity&                m_cHeightFieldEntity;
      
      btTerrainShape*                    m_pcCollisionShape; 
   };

}

#endif
