/**
 * @file <argos3/plugins/robots/robot/simulator/dynamics2d_robot_model.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef DYNAMICS2D_ROBOT_MODEL_H
#define DYNAMICS2D_ROBOT_MODEL_H

#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_model.h>
#include <argos3/plugins/robots/robot/simulator/robot_entity.h>

namespace argos {

   class CDynamics2DRobotModel : public CDynamics2DModel {

   public:

      CDynamics2DRobotModel(CDynamics2DEngine& c_engine,
                                 CRobotEntity& c_entity);
      virtual ~CDynamics2DRobotModel();
      
      virtual bool CheckIntersectionWithRay(Real& f_t_on_ray,
                                            const CRay3& c_ray) const;

      virtual bool MoveTo(const CVector3& c_position,
                          const CQuaternion& c_orientation,
                          bool b_check_only = false);

      virtual void Reset();

      virtual void CalculateBoundingBox();

      virtual void UpdateEntityStatus();
      virtual void UpdateFromEntityStatus();

      virtual bool IsCollidingWithSomething() const;

   private:

      CRobotEntity& m_cRobotEntity;
   };

}

#endif
