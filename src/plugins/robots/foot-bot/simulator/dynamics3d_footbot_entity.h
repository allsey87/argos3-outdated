/**
 * @file <argos3/plugins/robots/foot-bot/simulator/dynamics3d_footbot_entity.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef DYNAMICS3D_FOOTBOT_ENTITY_H
#define DYNAMICS3D_FOOTBOT_ENTITY_H

#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_entity.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

namespace argos {

   class CDynamics3DFootBotEntity : public CDynamics3DEntity {

   public:

      CDynamics3DFootBotEntity(CDynamics3DEngine& c_engine,
                               CFootBotEntity& c_entity);
      virtual ~CDynamics3DFootBotEntity();
      
      virtual bool CheckIntersectionWithRay(Real& f_t_on_ray,
                                            const CRay3& c_ray) const;

      virtual bool MoveTo(const CVector3& c_position,
                          const CQuaternion& c_orientation,
                          bool b_check_only = false);

      virtual void Reset();

      virtual void UpdateEntityStatus();
      virtual void UpdateFromEntityStatus();

      virtual void CalculateBoundingBox();

   private:

      CFootBotEntity& m_cFootBotEntity;
      CWheeledEntity& m_cWheeledEntity;
      
      Real m_fCurrentWheelVelocityFromSensor[2];
      
      /******** Collision Shapes - shared! **********/
      static btBoxShape          m_cBaseCollisionShape;
      static btCylinderShape     m_cWheelCollisionShape;
      static btSphereShape       m_cPivotCollisionShape;
      static btCylinderShape     m_cBodyCollisionShape;
      
      /**************** Motion States ****************/
      btDefaultMotionState*      m_pcBaseMotionState;
      btDefaultMotionState*      m_pcLeftWheelMotionState;
      btDefaultMotionState*      m_pcRightWheelMotionState;
      btDefaultMotionState*      m_pcFrontPivotMotionState;
      btDefaultMotionState*      m_pcRearPivotMotionState;
      btDefaultMotionState*      m_pcBodyMotionState;
      
      /**************** Rigid Bodies ****************/
      btRigidBody*               m_pcBaseRigidBody;
      btRigidBody*               m_pcLeftWheelRigidBody;
      btRigidBody*               m_pcRightWheelRigidBody;
      btRigidBody*               m_pcFrontPivotRigidBody;
      btRigidBody*               m_pcRearPivotRigidBody;
      btRigidBody*               m_pcBodyRigidBody;
      
      /**************** Constraints ****************/
      btHingeConstraint*         m_pcLeftWheelToBaseConstraint;
      btHingeConstraint*         m_pcRightWheelToBaseConstraint;
      btHingeConstraint*         m_pcBaseToBodyConstraint;
      btPoint2PointConstraint*   m_pcFrontPivotToBaseConstraint;
      btPoint2PointConstraint*   m_pcRearPivotToBaseConstraint;
   };
}

#endif
