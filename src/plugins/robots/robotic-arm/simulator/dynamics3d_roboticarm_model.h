/**
 * @file <argos3/plugins/robots/robotic-arm/simulator/dynamics3d_roboticarm_model.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef DYNAMICS3D_ROBOTICARM_MODEL_H
#define DYNAMICS3D_ROBOTICARM_MODEL_H

#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_model.h>
#include <argos3/plugins/robots/robotic-arm/simulator/roboticarm_entity.h>

namespace argos {

   class CDynamics3DRoboticArmModel : public CDynamics3DModel {

   public:

      CDynamics3DRoboticArmModel(CDynamics3DEngine& c_engine,
                                 CRoboticArmEntity& c_entity);
      virtual ~CDynamics3DRoboticArmModel();
      
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

      CRoboticArmEntity&      m_cRoboticArmEntity;
      CLinkEquippedEntity&    m_cLinkEquippedEntity;
      
      std::vector<btBoxShape*> m_vecLinkCollisionShapes;
      std::vector<btDefaultMotionState*> m_vecLinkMotionStates;
      std::vector<btRigidBody*> m_vecLinkRigidBodies;
      std::vector<btHingeConstraint*> m_vecLinkConstraints;
   };
}

#endif
