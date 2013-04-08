/**
 * @file <argos3/plugins/robots/robot/simulator/dynamics2d_robot_model.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "dynamics2d_robot_model.h"
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_engine.h>

namespace argos {

   /****************************************/
   /****************************************/

   CDynamics2DRobotModel::CDynamics2DRobotModel(CDynamics2DEngine& c_engine,
                                                          CRobotEntity& c_entity) :
      CDynamics2DModel(c_engine, c_entity.GetEmbodiedEntity()),
      m_cRobotEntity(c_entity) {}

   /****************************************/
   /****************************************/

   CDynamics2DRobotModel::~CDynamics2DRobotModel() {}

   /****************************************/
   /****************************************/

   bool CDynamics2DRobotModel::CheckIntersectionWithRay(Real& f_t_on_ray,
                                                           const CRay3& c_ray) const {
      return false;
   }

   /****************************************/
   /****************************************/

   bool CDynamics2DRobotModel::MoveTo(const CVector3& c_position,
                                         const CQuaternion& c_orientation,
                                         bool b_check_only) {
      return true;
   }

   /****************************************/
   /****************************************/

   void CDynamics2DRobotModel::Reset() {}

   /****************************************/
   /****************************************/

   void CDynamics2DRobotModel::CalculateBoundingBox() {
      /* @todo Implement CDynamics2DRobotModel::CalculateBoundingBox() */
   }

   /****************************************/
   /****************************************/

   void CDynamics2DRobotModel::UpdateEntityStatus() {}

   /****************************************/
   /****************************************/

   void CDynamics2DRobotModel::UpdateFromEntityStatus() {}
   
   /****************************************/
   /****************************************/

   bool CDynamics2DRobotModel::IsCollidingWithSomething() const {
      /* @todo Implement CDynamics2DBoxEntity::IsCollidingWithSomething() */
      return false;
      //return cpSpaceShapeQuery(m_cDyn2DEngine.GetPhysicsSpace(), m_ptShape, NULL, NULL) > 0;
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_DYNAMICS2D_OPERATIONS_ON_ENTITY(CRobotEntity, CDynamics2DRobotModel);

   /****************************************/
   /****************************************/

}

