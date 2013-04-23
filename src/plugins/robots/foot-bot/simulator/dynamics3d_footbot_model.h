/**
 * @file <argos3/plugins/robots/foot-bot/simulator/dynamics3d_footbot_model.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef DYNAMICS3D_FOOTBOT_MODEL_H
#define DYNAMICS3D_FOOTBOT_MODEL_H

#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_model.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

namespace argos {

   class CDynamics3DFootBotModel : public CDynamics3DModel {

   public:
      struct Body { 
         enum T {
            CHASSIS     = 0,
            LEFT_WHEEL  = 1,
            RIGHT_WHEEL = 2,
            FRONT_PIVOT = 3,
            REAR_PIVOT  = 4
         };
      };

   public:

      CDynamics3DFootBotModel(CDynamics3DEngine& c_engine,
                               CFootBotEntity& c_entity);
      virtual ~CDynamics3DFootBotModel();

      virtual void UpdateEntityStatus();
      virtual void UpdateFromEntityStatus();

   protected:

      virtual btTransform GetModelCoordinates() const;

   private:

      CFootBotEntity& m_cFootBotEntity;
      CWheeledEntity& m_cWheeledEntity;
      
      Real m_pfCurrentWheelVelocityFromSensor[2];
      
      /******** Shared Transforms **********/
      static btTransform                  m_cBatterySocketTransform;
      static btTransform                  m_cBaseModuleTransform;
      static btTransform                  m_cLeftWheelTransform;
      static btTransform                  m_cRightWheelTransform; 
      static btTransform                  m_cFrontPivotTransform;
      static btTransform                  m_cRearPivotTransform;
      static btTransform                  m_cChassisTransform;
      
      /******** Shared Collision Shapes **********/
      static btBoxShape                   m_cBatterySocketCollisionShape;
      static btCylinderShape              m_cBaseModuleCollisionShape;
      static btCylinderShape              m_cWheelCollisionShape;
      static btSphereShape                m_cPivotCollisionShape;
      static btCompoundShape              m_cChassisCollisionShape;
      
      /**************** Constraints ****************/
      btHingeConstraint*                  m_pcLeftWheelToChassisConstraint;
      btHingeConstraint*                  m_pcRightWheelToChassisConstraint;
      btPoint2PointConstraint*            m_pcFrontPivotToChassisConstraint;
      btPoint2PointConstraint*            m_pcRearPivotToChassisConstraint;
   };
}

#endif
