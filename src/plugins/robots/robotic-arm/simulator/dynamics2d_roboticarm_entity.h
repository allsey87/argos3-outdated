/**
 * @file <argos3/plugins/robots/robotic-arm/simulator/dynamics2d_roboticarm_entity.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef DYNAMICS2D_ROBOTICARM_ENTITY_H
#define DYNAMICS2D_ROBOTICARM_ENTITY_H

#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_entity.h>
#include <argos3/plugins/robots/robotic-arm/simulator/roboticarm_entity.h>

namespace argos {

   class CDynamics2DRoboticArmEntity : public CDynamics2DEntity {

   public:

      CDynamics2DRoboticArmEntity(CDynamics2DEngine& c_engine,
                               CRoboticArmEntity& c_entity);
      virtual ~CDynamics2DRoboticArmEntity();
      
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

      CRoboticArmEntity& m_cRoboticArmEntity;
   };

}

#endif
