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
      
      Real m_pfCurrentWheelVelocityFromSensor[2];
      
      /******** Shared Transforms **********/
      static btTransform                  m_cLowerBodyTransform;
      static btTransform                  m_cUpperBodyTransform;
      static btTransform                  m_cBodyTransform;
      static btTransform                  m_cLeftWheelTransform;
      static btTransform                  m_cRightWheelTransform; 
      static btTransform                  m_cFrontPivotTransform;
      static btTransform                  m_cRearPivotTransform;
      
      /******** Shared Collision Shapes **********/
      static btBoxShape                   m_cLowerBodyCollisionShape;
      static btCylinderShape              m_cUpperBodyCollisionShape;
      static btCompoundShape              m_cBodyCollisionShape;
      static btCylinderShape              m_cWheelCollisionShape;
      static btSphereShape                m_cPivotCollisionShape;
      
      
      /**************** Motion States ****************/
      btDefaultMotionState*               m_pcLeftWheelMotionState;
      btDefaultMotionState*               m_pcRightWheelMotionState;
      btDefaultMotionState*               m_pcFrontPivotMotionState;
      btDefaultMotionState*               m_pcRearPivotMotionState;
      btDefaultMotionState*               m_pcBodyMotionState;
      
      /**************** Rigid Bodies ****************/
      btRigidBody*                        m_pcLeftWheelRigidBody;
      btRigidBody*                        m_pcRightWheelRigidBody;
      btRigidBody*                        m_pcFrontPivotRigidBody;
      btRigidBody*                        m_pcRearPivotRigidBody;
      btRigidBody*                        m_pcBodyRigidBody;
      
      /**************** Constraints ****************/
      btHingeConstraint*                  m_pcLeftWheelToBodyConstraint;
      btHingeConstraint*                  m_pcRightWheelToBodyConstraint;
      btHingeConstraint*                  m_pcBodyToBodyConstraint;
      btPoint2PointConstraint*            m_pcFrontPivotToBodyConstraint;
      btPoint2PointConstraint*            m_pcRearPivotToBodyConstraint;
   };
}

#endif
