/**
 * @file <argos3/plugins/robots/foot-bot/simulator/dynamics3d_footbot_model.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "dynamics3d_footbot_model.h"

#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_engine.h>
#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_body.h>

namespace argos {

   static const Real FOOTBOT_WHEEL_RADIUS          =  0.029112741f;

   static const Real FOOTBOT_BATT_W                =  0.081662841f;
   static const Real FOOTBOT_BATT_H                =  FOOTBOT_WHEEL_RADIUS * 2;
   static const Real FOOTBOT_BATT_L                =  0.150302467f;
   static const Real FOOTBOT_BATT_GROUND_OFFSET    =  0.006000000f;
   static const Real FOOTBOT_BASEMODULE_RADIUS     =  0.085036758f;
   static const Real FOOTBOT_BASEMODULE_HEIGHT     =  0.072000000f;
   static const Real FOOTBOT_CHASSIS_MASS          =  0.100000000f;

   static const Real FOOTBOT_PIVOT_RADIUS          =  FOOTBOT_WHEEL_RADIUS;
   static const Real FOOTBOT_PIVOT_HALF_DISTANCE   =  0.050000000f;
   static const Real FOOTBOT_PIVOT_MASS            =  0.100000000f;
   static const Real FOOTBOT_PIVOT_Y_OFFSET        =  FOOTBOT_PIVOT_RADIUS + FOOTBOT_BATT_GROUND_OFFSET;   

   static const Real FOOTBOT_WHEEL_THICKNESS       =  0.022031354f;
   static const Real FOOTBOT_WHEEL_HALF_DISTANCE   =  0.070000000f;
   static const Real FOOTBOT_WHEEL_MASS            =  0.100000000f;
   static const Real FOOTBOT_WHEEL_MOTOR_IMPULSE   =  15.00000f;
   static const Real FOOTBOT_WHEEL_Y_OFFSET        =  FOOTBOT_WHEEL_RADIUS + FOOTBOT_BATT_GROUND_OFFSET;
   //  static const Real FOOTBOT_CENTER_OF_MASS_OFFSET =  FOOTBOT_BATT_H + FOOTBOT_BATT_GROUND_OFFSET;

   enum EFootbotWheels {
      FOOTBOT_LEFT_WHEEL  = 0,
      FOOTBOT_RIGHT_WHEEL = 1
   };

   /****************************************/
   /****************************************/

   // Shared static transforms describing the FootBot geometric layout
   const btTransform CDynamics3DFootBotModel::m_cBatterySocketCompoundOffset(
      btQuaternion(0.0f, 0.0f, 0.0f, 1.0f),
      btVector3(0.0f, -FOOTBOT_BATT_H * 0.5f, 0.0f));

   const btTransform CDynamics3DFootBotModel::m_cBaseModuleCompoundOffset(
      btQuaternion(0.0f, 0.0f, 0.0f, 1.0f),
      btVector3(0.0f, FOOTBOT_BASEMODULE_HEIGHT * 0.5f, 0.0f));

   const btTransform CDynamics3DFootBotModel::m_cChassisGeometricOffset(
      btQuaternion(0.0f, 0.0f, 0.0f, 1.0f),
      btVector3(0.0f, -FOOTBOT_BATT_H, 0.0f)); 

   const btTransform CDynamics3DFootBotModel::m_cChassisPositionalOffset(
      btQuaternion(0.0f, 0.0f, 0.0f, 1.0f),
      btVector3(0.0f, FOOTBOT_BATT_GROUND_OFFSET, 0.0f));


   const btTransform CDynamics3DFootBotModel::m_cWheelGeometricOffset(
      btQuaternion(0.0f, 0.0f, 0.0f, 1.0f),
      btVector3(0.0f, -FOOTBOT_WHEEL_THICKNESS * 0.5f, 0.0f));

   const btTransform CDynamics3DFootBotModel::m_cLeftWheelPositionalOffset(
      btQuaternion(btVector3(1.0f, 0.0f, 0.0f), -ARGOS_PI * 0.5f),
      btVector3(0.0f, FOOTBOT_WHEEL_RADIUS, -FOOTBOT_WHEEL_HALF_DISTANCE + FOOTBOT_WHEEL_THICKNESS * 0.5f));

   const btTransform CDynamics3DFootBotModel::m_cRightWheelPositionalOffset(
      btQuaternion(btVector3(1.0f, 0.0f, 0.0f), -ARGOS_PI * 0.5f),
      btVector3(0.0f, FOOTBOT_WHEEL_RADIUS, FOOTBOT_WHEEL_HALF_DISTANCE + FOOTBOT_WHEEL_THICKNESS * 0.5f));


   const btTransform CDynamics3DFootBotModel::m_cPivotGeometricOffset(
      btQuaternion(0.0f, 0.0f, 0.0f, 1.0f),
      btVector3(0.0f, -FOOTBOT_PIVOT_RADIUS, 0.0f));

   const btTransform CDynamics3DFootBotModel::m_cFrontPivotPositionalOffset(
      btQuaternion(0.0f, 0.0f, 0.0f, 1.0f),
      btVector3(FOOTBOT_PIVOT_HALF_DISTANCE, 0.0f, 0.0f));

   const btTransform CDynamics3DFootBotModel::m_cRearPivotPositionalOffset(
      btQuaternion(0.0f, 0.0f, 0.0f, 1.0f),
      btVector3(-FOOTBOT_PIVOT_HALF_DISTANCE, 0.0f, 0.0f));

   const btTransform CDynamics3DFootBotModel::m_cChassisToLeftAxleTransform(
      btQuaternion(0.0f, 0.0f, 0.0f, 1.0f),
      btVector3(0.0f, -FOOTBOT_WHEEL_Y_OFFSET, -FOOTBOT_WHEEL_HALF_DISTANCE));

   const btTransform CDynamics3DFootBotModel::m_cLeftWheelToLeftAxleTransform(
      btQuaternion(btVector3(1.0f, 0.0f, 0.0f), ARGOS_PI * 0.5f),
      btVector3(0.0f, 0.0f, 0.0f));

   const btTransform CDynamics3DFootBotModel::m_cChassisToRightAxleTransform(
      btQuaternion(0.0f, 0.0f, 0.0f, 1.0f),
      btVector3(0.0f, -FOOTBOT_WHEEL_Y_OFFSET, FOOTBOT_WHEEL_HALF_DISTANCE));

   const btTransform CDynamics3DFootBotModel::m_cRightWheelToRightAxleTransform(
      btQuaternion(btVector3(1.0f, 0.0f, 0.0f), ARGOS_PI * 0.5f),
      btVector3(0.0f, 0.0f, 0.0f));

   const btTransform CDynamics3DFootBotModel::m_cChassisToFrontPivotTransform(
      btQuaternion(0.0f, 0.0f, 0.0f, 1.0f),
      btVector3(FOOTBOT_PIVOT_HALF_DISTANCE, -FOOTBOT_PIVOT_Y_OFFSET, 0.0f));

   const btTransform CDynamics3DFootBotModel::m_cChassisToRearPivotTransform(
      btQuaternion(0.0f, 0.0f, 0.0f, 1.0f),
      btVector3(-FOOTBOT_PIVOT_HALF_DISTANCE, -FOOTBOT_PIVOT_Y_OFFSET, 0.0f));


   // Shared static collsion shapes describing the FootBot geometry
   btBoxShape CDynamics3DFootBotModel::m_cBatterySocketCollisionShape(
      btVector3(FOOTBOT_BATT_L, FOOTBOT_BATT_H, FOOTBOT_BATT_W) * 0.5f);

   btCylinderShape CDynamics3DFootBotModel::m_cBaseModuleCollisionShape(
      btVector3(FOOTBOT_BASEMODULE_RADIUS, FOOTBOT_BASEMODULE_HEIGHT * 0.5f, FOOTBOT_BASEMODULE_RADIUS));

   // Initialiser in constructor - see below
   btCompoundShape CDynamics3DFootBotModel::m_cChassisCollisionShape;

   btCylinderShape CDynamics3DFootBotModel::m_cWheelCollisionShape(
      btVector3(FOOTBOT_WHEEL_RADIUS, FOOTBOT_WHEEL_THICKNESS * 0.5f, FOOTBOT_WHEEL_RADIUS));

   btSphereShape CDynamics3DFootBotModel::m_cPivotCollisionShape(FOOTBOT_PIVOT_RADIUS);

   /****************************************/
   /****************************************/

   CDynamics3DFootBotModel::CDynamics3DFootBotModel(CDynamics3DEngine& c_engine,
                                                    CFootBotEntity& c_entity) :
      CDynamics3DModel(c_engine, c_entity.GetEmbodiedEntity(), c_entity.GetId()),
      m_cFootBotEntity(c_entity),
      m_cWheeledEntity(m_cFootBotEntity.GetWheeledEntity()) {

      if(m_cChassisCollisionShape.getNumChildShapes() == 0) {
         m_cChassisCollisionShape.addChildShape(m_cBatterySocketCompoundOffset, &m_cBatterySocketCollisionShape);
         m_cChassisCollisionShape.addChildShape(m_cBaseModuleCompoundOffset, &m_cBaseModuleCollisionShape);
      }

      /* create the bodies */
      m_vecLocalBodies.push_back(new CDynamics3DBody(this,
                                                     "chassis", 
                                                     &m_cChassisCollisionShape,
                                                     m_cChassisPositionalOffset,
                                                     m_cChassisGeometricOffset,
                                                     FOOTBOT_CHASSIS_MASS));

      m_vecLocalBodies.push_back(new CDynamics3DBody(this,
                                                     "left-wheel", 
                                                     &m_cWheelCollisionShape,
                                                     m_cLeftWheelPositionalOffset,
                                                     m_cWheelGeometricOffset,
                                                     FOOTBOT_WHEEL_MASS));

      m_vecLocalBodies.push_back(new CDynamics3DBody(this,
                                                     "right-wheel",
                                                     &m_cWheelCollisionShape,
                                                     m_cRightWheelPositionalOffset,
                                                     m_cWheelGeometricOffset,
                                                     FOOTBOT_WHEEL_MASS));

      m_vecLocalBodies.push_back(new CDynamics3DBody(this,
                                                     "front-pivot",
                                                     &m_cPivotCollisionShape,
                                                     m_cFrontPivotPositionalOffset,
                                                     m_cPivotGeometricOffset,
                                                     FOOTBOT_PIVOT_MASS));
 
      m_vecLocalBodies.push_back(new CDynamics3DBody(this,
                                                     "rear-pivot",
                                                     &m_cPivotCollisionShape,
                                                     m_cRearPivotPositionalOffset,
                                                     m_cPivotGeometricOffset,
                                                     FOOTBOT_PIVOT_MASS));

      /* create the joints */
      m_vecLocalJoints.push_back(new CDynamics3DJoint("left-wheel:chassis",
                                                      *m_vecLocalBodies[Body::LEFT_WHEEL],
                                                      *m_vecLocalBodies[Body::CHASSIS],
                                                      m_cLeftWheelToLeftAxleTransform,
                                                      m_cChassisToLeftAxleTransform,
                                                      CDynamics3DJoint::m_cLockAxes,
                                                      CDynamics3DJoint::m_cFreeAxisZ,
                                                      CDynamics3DJoint::SJointActuators(),
                                                      CDynamics3DJoint::SJointActuators(
                                                         CDynamics3DJoint::SJointActuators::SActuator(),
                                                         CDynamics3DJoint::SJointActuators::SActuator(),
                                                         CDynamics3DJoint::SJointActuators::SActuator(
                                                            true, 
                                                            FOOTBOT_WHEEL_MOTOR_IMPULSE,
                                                            0.0f)),
                                                      true,
                                                      true));
                                                     
      m_vecLocalJoints.push_back(new CDynamics3DJoint("right-wheel:chassis",
                                                      *m_vecLocalBodies[Body::RIGHT_WHEEL],
                                                      *m_vecLocalBodies[Body::CHASSIS],
                                                      m_cRightWheelToRightAxleTransform,
                                                      m_cChassisToRightAxleTransform,
                                                      CDynamics3DJoint::m_cLockAxes,
                                                      CDynamics3DJoint::m_cFreeAxisZ,
                                                      CDynamics3DJoint::SJointActuators(),
                                                      CDynamics3DJoint::SJointActuators(
                                                         CDynamics3DJoint::SJointActuators::SActuator(),
                                                         CDynamics3DJoint::SJointActuators::SActuator(),
                                                         CDynamics3DJoint::SJointActuators::SActuator(
                                                            true, 
                                                            FOOTBOT_WHEEL_MOTOR_IMPULSE,
                                                            0.0f)),
                                                      true,
                                                      true));                                                     

      m_vecLocalJoints.push_back(new CDynamics3DJoint("front-pivot:chassis",
                                                      *m_vecLocalBodies[Body::FRONT_PIVOT],
                                                      *m_vecLocalBodies[Body::CHASSIS],
                                                      btTransform::getIdentity(),
                                                      m_cChassisToFrontPivotTransform,
                                                      CDynamics3DJoint::m_cLockAxes,
                                                      CDynamics3DJoint::m_cFreeAxisXYZ,
                                                      CDynamics3DJoint::SJointActuators(),
                                                      CDynamics3DJoint::SJointActuators(),
                                                      true,
                                                      true));                                                     

      m_vecLocalJoints.push_back(new CDynamics3DJoint("rear-pivot:chassis",
                                                      *m_vecLocalBodies[Body::REAR_PIVOT],
                                                      *m_vecLocalBodies[Body::CHASSIS],
                                                      btTransform::getIdentity(),
                                                      m_cChassisToRearPivotTransform,
                                                      CDynamics3DJoint::m_cLockAxes,
                                                      CDynamics3DJoint::m_cFreeAxisXYZ,
                                                      CDynamics3DJoint::SJointActuators(),
                                                      CDynamics3DJoint::SJointActuators(),
                                                      true,
                                                      true));                                                     

     /* move the model to the specified coordinates */
     SetModelCoordinates(btTransform(ARGoSToBullet(GetEmbodiedEntity().GetInitOrientation()),
                                     ARGoSToBullet(GetEmbodiedEntity().GetInitPosition())));
   }

   /****************************************/
   /****************************************/

   void CDynamics3DFootBotModel::Reset() {
      //@todo dissolve this method, it does nothing and is provided by the base class

      /* call the CDynamics3DModel::Reset method to reset
         and reposition the bodies and joints */
      CDynamics3DModel::Reset();
   }

   /****************************************/
   /****************************************/

   void CDynamics3DFootBotModel::UpdateEntityStatus() {
      /* Update footbot position and orientation based on the location of the reference body */
      const btTransform& cEntityUpdateTransform = GetModelCoordinates();

      GetEmbodiedEntity().SetPosition(BulletToARGoS(cEntityUpdateTransform.getOrigin()));
      GetEmbodiedEntity().SetOrientation(BulletToARGoS(cEntityUpdateTransform.getRotation()));

      /* Update components */
      m_cFootBotEntity.UpdateComponents();
   }

   /****************************************/
   /****************************************/

   void CDynamics3DFootBotModel::UpdateFromEntityStatus() {

      /* Get wheel speeds from entity */
      const Real* m_pfCurrentWheelVelocityFromSensor = m_cWheeledEntity.GetWheelVelocities();

      /* Do we want to move? */
      if((m_pfCurrentWheelVelocityFromSensor[FOOTBOT_LEFT_WHEEL] != 0.0f) ||
         (m_pfCurrentWheelVelocityFromSensor[FOOTBOT_RIGHT_WHEEL] != 0.0f)) {

         Real fLeftWheelVelocity, fRightWheelVelocity;

         fLeftWheelVelocity  = m_pfCurrentWheelVelocityFromSensor[FOOTBOT_LEFT_WHEEL] / FOOTBOT_WHEEL_RADIUS;
         fRightWheelVelocity = m_pfCurrentWheelVelocityFromSensor[FOOTBOT_RIGHT_WHEEL] / FOOTBOT_WHEEL_RADIUS;

         /* Bullet automatically arranges bodies connected by constraints into islands,
            when we activate one body in the island, all of the bodies become activated */
         m_vecLocalBodies[Body::CHASSIS]->ActivateRigidBody();
         
         /* Write the motor target velocities to the joints */
         m_vecLocalJoints[Joint::LEFT_WHEEL_TO_CHASSIS]->SetActuatorTargetVelocity(
            CDynamics3DJoint::ANGULAR_Z,
            fLeftWheelVelocity);

         m_vecLocalJoints[Joint::RIGHT_WHEEL_TO_CHASSIS]->SetActuatorTargetVelocity(
            CDynamics3DJoint::ANGULAR_Z,
            fRightWheelVelocity);
      }
      else {
         
         /* Write a target velocity of zero to the joints */
         m_vecLocalJoints[Joint::LEFT_WHEEL_TO_CHASSIS]->SetActuatorTargetVelocity(
            CDynamics3DJoint::ANGULAR_Z,
            0.0f);

         m_vecLocalJoints[Joint::RIGHT_WHEEL_TO_CHASSIS]->SetActuatorTargetVelocity(
            CDynamics3DJoint::ANGULAR_Z,
            0.0f);
      }
   }

   /****************************************/
   /****************************************/

   btTransform CDynamics3DFootBotModel::GetModelCoordinates() const {
      return m_vecLocalBodies[Body::CHASSIS]->GetMotionStateTransform() *
         m_cChassisPositionalOffset.inverse();
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_DYNAMICS3D_OPERATIONS_ON_ENTITY(CFootBotEntity, CDynamics3DFootBotModel);

}
