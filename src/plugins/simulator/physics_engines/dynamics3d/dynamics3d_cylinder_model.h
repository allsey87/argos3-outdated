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
                     CCylinderEntity& c_cylinder);
      virtual ~CDynamics3DCylinderModel();
      
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

      CCylinderEntity&                m_cCylinderEntity;
      
      btCylinderShape*                m_pcCylinderBaseShape;
      btCompoundShape*           m_pcCylinderCollisionShape;
      btDefaultMotionState*      m_pcCylinderMotionState;
      btRigidBody*               m_pcCylinderRigidBody;
   };
}

#endif
