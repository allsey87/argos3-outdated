/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_terrain_entity.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef DYNAMICS3D_TERRAIN_ENTITY_H
#define DYNAMICS3D_TERRAIN_ENTITY_H

namespace argos {
   class CDynamics3DEngine;
   class CDynamics3DTerrainEntity;
}

#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_entity.h>
#include <argos3/plugins/simulator/entities/terrain_entity.h>


namespace argos {

   class CDynamics3DTerrainEntity : public CDynamics3DEntity {

   public:
      
      CDynamics3DTerrainEntity(CDynamics3DEngine& c_engine,
                     CTerrainEntity& c_Terrain);
      virtual ~CDynamics3DTerrainEntity();
      
      virtual bool CheckIntersectionWithRay(Real& f_t_on_ray,
                                            const CRay3& c_ray) const;

      virtual bool MoveTo(const CVector3& c_position,
                          const CQuaternion& c_orientation,
                          bool b_check_only = false) {
         return false; // moving of terrain is not supported!
      }

      virtual void Reset();

      virtual void UpdateEntityStatus() {}
      virtual void UpdateFromEntityStatus() {}
      
      virtual std::vector<btRigidBody*>& GetRigidBodies() {
         return m_vecRigidBodies;
      }
      

   private:

      CTerrainEntity&                m_cTerrainEntity;
      
      std::vector<btRigidBody*>      m_vecRigidBodies;
      
      btTerrainShape*                m_pcCollisionShape; 
   };

}

#endif
