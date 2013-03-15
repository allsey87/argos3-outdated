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
      
      virtual bool CheckIntersectionWithRay(Real& f_t_on_ray,
                                            const CRay3& c_ray) const;

      virtual bool MoveTo(const CVector3& c_position,
                          const CQuaternion& c_orientation,
                          bool b_check_only = false);

      virtual void Reset();

      virtual void UpdateEntityStatus();
      virtual void UpdateFromEntityStatus() {}

      virtual void CalculateBoundingBox();
      
      virtual bool IsCollidingWithSomething() const { return false; }
      
   private:

      CBoxEntity&                m_cBoxEntity;
      
      btTransform*               m_pcBoxTransform;
      btBoxShape*                m_pcBoxCollisionShape;
      btDefaultMotionState*      m_pcBoxMotionState;
      btRigidBody*               m_pcBoxRigidBody;
   };
}

#endif
