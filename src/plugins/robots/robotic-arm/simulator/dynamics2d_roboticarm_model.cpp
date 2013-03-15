/**
 * @file <argos3/plugins/robots/robotic-arm/simulator/dynamics2d_roboticarm_model.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "dynamics2d_roboticarm_model.h"
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_engine.h>

namespace argos {

   /****************************************/
   /****************************************/

   CDynamics2DRoboticArmModel::CDynamics2DRoboticArmModel(CDynamics2DEngine& c_engine,
                                                          CRoboticArmEntity& c_entity) :
      CDynamics2DModel(c_engine, c_entity.GetEmbodiedEntity()),
      m_cRoboticArmEntity(c_entity) {}

   /****************************************/
   /****************************************/

   CDynamics2DRoboticArmModel::~CDynamics2DRoboticArmModel() {}

   /****************************************/
   /****************************************/

   bool CDynamics2DRoboticArmModel::CheckIntersectionWithRay(Real& f_t_on_ray,
                                                           const CRay3& c_ray) const {
      return false;
   }

   /****************************************/
   /****************************************/

   bool CDynamics2DRoboticArmModel::MoveTo(const CVector3& c_position,
                                         const CQuaternion& c_orientation,
                                         bool b_check_only) {
      return true;
   }

   /****************************************/
   /****************************************/

   void CDynamics2DRoboticArmModel::Reset() {}

   /****************************************/
   /****************************************/

   void CDynamics2DRoboticArmModel::CalculateBoundingBox() {
      /* @todo Implement CDynamics2DBoxEntity::CalculateBoundingBox() */
      // GetBoundingBox().MinCorner.SetX(m_ptShape->bb.l);
      // GetBoundingBox().MinCorner.SetY(m_ptShape->bb.b);
      // GetBoundingBox().MaxCorner.SetX(m_ptShape->bb.r);
      // GetBoundingBox().MaxCorner.SetY(m_ptShape->bb.t);
   }

   /****************************************/
   /****************************************/

   void CDynamics2DRoboticArmModel::UpdateEntityStatus() {}

   /****************************************/
   /****************************************/

   void CDynamics2DRoboticArmModel::UpdateFromEntityStatus() {}
   
   /****************************************/
   /****************************************/

   bool CDynamics2DRoboticArmModel::IsCollidingWithSomething() const {
      /* @todo Implement CDynamics2DBoxEntity::IsCollidingWithSomething() */
      return false;
      //return cpSpaceShapeQuery(m_cDyn2DEngine.GetPhysicsSpace(), m_ptShape, NULL, NULL) > 0;
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_DYNAMICS2D_OPERATIONS_ON_ENTITY(CRoboticArmEntity, CDynamics2DRoboticArmModel);

   /****************************************/
   /****************************************/

}

