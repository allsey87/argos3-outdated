/**
 * @file <argos3/plugins/robots/robotic-arm/simulator/dynamics3d_roboticarm_entity.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef DYNAMICS3D_ROBOTICARM_ENTITY_H
#define DYNAMICS3D_ROBOTICARM_ENTITY_H

#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_entity.h>
#include <argos3/plugins/robots/robotic-arm/simulator/roboticarm_entity.h>

namespace argos {

   class CDynamics3DRoboticArmEntity : public CDynamics3DEntity {

   public:

      CDynamics3DRoboticArmEntity(CDynamics3DEngine& c_engine,
                                  CRoboticArmEntity& c_entity);
      virtual ~CDynamics3DRoboticArmEntity();
      
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
      
      btSphereShape*          m_pcMountingPointCollisionShape;
      btDefaultMotionState*   m_pcMountingPointMotionState;
      btRigidBody*            m_pcMountingPointRigidBody;
   };
}

#endif
