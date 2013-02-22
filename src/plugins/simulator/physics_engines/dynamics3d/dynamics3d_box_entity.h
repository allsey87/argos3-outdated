/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_box_entity.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef DYNAMICS3D_BOX_ENTITY_H
#define DYNAMICS3D_BOX_ENTITY_H

namespace argos {
   class CDynamics3DEngine;
   class CDynamics3DBoxEntity;
}

#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_entity.h>
#include <argos3/plugins/simulator/entities/box_entity.h>


namespace argos {

   class CDynamics3DBoxEntity : public CDynamics3DEntity {

   public:
      
      CDynamics3DBoxEntity(CDynamics3DEngine& c_engine,
                     CBoxEntity& c_box);
      virtual ~CDynamics3DBoxEntity();
      
      virtual bool CheckIntersectionWithRay(Real& f_t_on_ray,
                                            const CRay3& c_ray) const;

      virtual bool MoveTo(const CVector3& c_position,
                          const CQuaternion& c_orientation,
                          bool b_check_only = false);

      virtual void Reset();

      virtual void UpdateEntityStatus();
      virtual void UpdateFromEntityStatus() {}
      
      virtual std::vector<btRigidBody*>& GetRigidBodies() {
         return m_vecRigidBodies;
      }
      

   private:

      CBoxEntity&                m_cBoxEntity;
      
      std::vector<btRigidBody*>  m_vecRigidBodies;
      
      btBoxShape*                m_pcCollisionShape; 
   };

}

#endif
