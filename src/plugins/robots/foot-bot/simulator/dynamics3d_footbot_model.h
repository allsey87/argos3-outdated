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
         const static UInt8 CHASSIS     = 0;
         const static UInt8 LEFT_WHEEL  = 1;
         const static UInt8 RIGHT_WHEEL = 2;
         const static UInt8 FRONT_PIVOT = 3;
         const static UInt8 REAR_PIVOT  = 4;
      };

      struct Joint {
         const static UInt8 LEFT_WHEEL_TO_CHASSIS = 0;
         const static UInt8 RIGHT_WHEEL_TO_CHASSIS = 1;
         const static UInt8 FRONT_PIVOT_TO_CHASSIS = 2;
         const static UInt8 REAR_PIVOT_TO_CHASSIS = 3;
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
      static btTransform                  m_cBatterySocketCompoundOffset;
      static btTransform                  m_cBaseModuleCompoundOffset;

      static btTransform                  m_cLeftWheelPositionalOffset;
      static btTransform                  m_cRightWheelPositionalOffset; 
      static btTransform                  m_cFrontPivotPositionalOffset;
      static btTransform                  m_cRearPivotPositionalOffset;
      static btTransform                  m_cChassisPositionalOffset;
      
      /******** Shared Collision Shapes **********/
      static btBoxShape                   m_cBatterySocketCollisionShape;
      static btCylinderShape              m_cBaseModuleCollisionShape;
      static btCylinderShape              m_cWheelCollisionShape;
      static btSphereShape                m_cPivotCollisionShape;
      static btCompoundShape              m_cChassisCollisionShape;
      
      /**************** Constraints ****************/
      btGeneric6DofConstraint*            m_pcLeftWheelToChassisConstraint;
      btGeneric6DofConstraint*            m_pcRightWheelToChassisConstraint;
      btGeneric6DofConstraint*            m_pcFrontPivotToChassisConstraint;
      btGeneric6DofConstraint*            m_pcRearPivotToChassisConstraint;
   };
}

#endif
