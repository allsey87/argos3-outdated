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
   static const Real FOOTBOT_WHEEL_THICKNESS       =  0.022031354f;
   static const Real FOOTBOT_WHEEL_HALF_DISTANCE   =  0.070000000f;
   static const Real FOOTBOT_WHEEL_MASS            =  0.100000000f;
   static const Real FOOTBOT_WHEEL_MOTOR_IMPULSE   =  15000.00000f;

   static const Real FOOTBOT_BATT_W                =  0.081662841f;
   static const Real FOOTBOT_BATT_H                =  FOOTBOT_WHEEL_RADIUS * 2;
   static const Real FOOTBOT_BATT_L                =  0.150302467f;
   static const Real FOOTBOT_BATT_GROUND_OFFSET    =  0.006000000f;
   static const Real FOOTBOT_BASEMODULE_RADIUS     =  0.085036758f;
   static const Real FOOTBOT_BASEMODULE_HEIGHT     =  0.072000000f;
   //static const Real FOOTBOT_BASEMODULE_Y_OFFSET   =  (FOOTBOT_BATT_H + FOOTBOT_BASEMODULE_HEIGHT) / 2.0f;
   static const Real FOOTBOT_BASEMODULE_MASS       =  0.100000000f;

   static const Real FOOTBOT_PIVOT_RADIUS          =  FOOTBOT_WHEEL_RADIUS;
   static const Real FOOTBOT_PIVOT_HALF_DISTANCE   =  0.050000000f;
   static const Real FOOTBOT_PIVOT_MASS            =  0.100000000f;
   
   static const Real FOOTBOT_CENTER_OF_MASS_OFFSET =  FOOTBOT_BATT_H + FOOTBOT_BATT_GROUND_OFFSET;
   static const Real FOOTBOT_WHEEL_Y_OFFSET        =  FOOTBOT_WHEEL_RADIUS + FOOTBOT_BATT_GROUND_OFFSET;
   static const Real FOOTBOT_PIVOT_Y_OFFSET        =  FOOTBOT_PIVOT_RADIUS + FOOTBOT_BATT_GROUND_OFFSET;

   enum EFootbotWheels {
      FOOTBOT_LEFT_WHEEL  = 0,
      FOOTBOT_RIGHT_WHEEL = 1
   };

  

   /****************************************/
   /****************************************/

   // Shared static transforms for bodies
   btTransform CDynamics3DFootBotModel::m_cLeftWheelTransform(
      btQuaternion(btVector3(1.0f, 0.0f, 0.0f), -ARGOS_PI * 0.5f),
      btVector3(0.0f, FOOTBOT_WHEEL_RADIUS, -FOOTBOT_WHEEL_HALF_DISTANCE + FOOTBOT_WHEEL_THICKNESS * 0.5f));

   btTransform CDynamics3DFootBotModel::m_cRightWheelTransform(
      btQuaternion(btVector3(1.0f, 0.0f, 0.0f), ARGOS_PI * 0.5f),
      btVector3(0.0f, FOOTBOT_WHEEL_RADIUS, FOOTBOT_WHEEL_HALF_DISTANCE - FOOTBOT_WHEEL_THICKNESS * 0.5f));

   btTransform CDynamics3DFootBotModel::m_cFrontPivotTransform(
      btQuaternion(0.0f, 0.0f, 0.0f, 1.0f),
      btVector3(FOOTBOT_PIVOT_HALF_DISTANCE, 0.0f, 0.0f));

   btTransform CDynamics3DFootBotModel::m_cRearPivotTransform(
      btQuaternion(0.0f, 0.0f, 0.0f, 1.0f),
      btVector3(-FOOTBOT_PIVOT_HALF_DISTANCE, 0.0f, 0.0f));

   btTransform CDynamics3DFootBotModel::m_cChassisTransform(
      btQuaternion(0.0f, 0.0f, 0.0f, 1.0f),
      btVector3(0.0f, FOOTBOT_BATT_GROUND_OFFSET, 0.0f));

   // Shared static transforms for compound body geometry
   btTransform CDynamics3DFootBotModel::m_cBatterySocketTransform(
      btQuaternion(0.0f, 0.0f, 0.0f, 1.0f),
      btVector3(0.0f, -FOOTBOT_BATT_H * 0.5f, 0.0f));

   btTransform CDynamics3DFootBotModel::m_cBaseModuleTransform(
      btQuaternion(0.0f, 0.0f, 0.0f, 1.0f),
      btVector3(0.0f, FOOTBOT_BASEMODULE_HEIGHT * 0.5f, 0.0f));

   // Shared static collision shapes
   btBoxShape CDynamics3DFootBotModel::m_cBatterySocketCollisionShape(
      btVector3(FOOTBOT_BATT_L, FOOTBOT_BATT_H, FOOTBOT_BATT_W) * 0.5f);

   btCylinderShape CDynamics3DFootBotModel::m_cBaseModuleCollisionShape(
      btVector3(FOOTBOT_BASEMODULE_RADIUS, FOOTBOT_BASEMODULE_HEIGHT * 0.5f, FOOTBOT_BASEMODULE_RADIUS));

   btCylinderShape CDynamics3DFootBotModel::m_cWheelCollisionShape(
      btVector3(FOOTBOT_WHEEL_RADIUS, FOOTBOT_WHEEL_THICKNESS * 0.5f, FOOTBOT_WHEEL_RADIUS));

   btSphereShape CDynamics3DFootBotModel::m_cPivotCollisionShape(FOOTBOT_PIVOT_RADIUS);

   // This collision shape requires init - see the constructor below!
   btCompoundShape CDynamics3DFootBotModel::m_cChassisCollisionShape;

   /****************************************/
   /****************************************/

   CDynamics3DFootBotModel::CDynamics3DFootBotModel(CDynamics3DEngine& c_engine,
                                                      CFootBotEntity& c_entity) :
      CDynamics3DModel(c_engine, c_entity.GetEmbodiedEntity()),
      m_cFootBotEntity(c_entity),
      m_cWheeledEntity(m_cFootBotEntity.GetWheeledEntity()) {

      if(m_cChassisCollisionShape.getNumChildShapes() == 0) {
         m_cChassisCollisionShape.addChildShape(m_cBatterySocketTransform, &m_cBatterySocketCollisionShape);
         m_cChassisCollisionShape.addChildShape(m_cBaseModuleTransform, &m_cBaseModuleCollisionShape);
      }

      /** Create the body **/
      btTransform chassisGeo(btQuaternion(0.0f, 0.0f, 0.0f, 1.0f), btVector3(0.0f, -FOOTBOT_BATT_H, 0.0f));
      
      m_vecLocalBodies.push_back(
         CDynamics3DBody::TNamedElement("chassis", new CDynamics3DBody(&m_cChassisCollisionShape,
                                                                       m_cChassisTransform,
                                                                       chassisGeo,
                                                                       FOOTBOT_BASEMODULE_MASS)));

      /** create the wheels **/
      btTransform wheelGeo(
         btQuaternion(0.0f, 0.0f, 0.0f, 1.0f),
         btVector3(0.0f, -FOOTBOT_WHEEL_THICKNESS * 0.5f, 0.0f));

      m_vecLocalBodies.push_back(
         CDynamics3DBody::TNamedElement("left-wheel", new CDynamics3DBody(&m_cWheelCollisionShape,
                                                                          m_cLeftWheelTransform,
                                                                          wheelGeo,
                                                                          FOOTBOT_WHEEL_MASS)));
      m_vecLocalBodies.push_back(
         CDynamics3DBody::TNamedElement("right-wheel", new CDynamics3DBody(&m_cWheelCollisionShape,
                                                                           m_cRightWheelTransform,
                                                                           wheelGeo,
                                                                           FOOTBOT_WHEEL_MASS)));

      /** create the wheels to body constraints **/
      m_pcLeftWheelToChassisConstraint = new btHingeConstraint(
         m_vecLocalBodies[Body::LEFT_WHEEL]->GetRigidBody(),
         m_vecLocalBodies[Body::CHASSIS]->GetRigidBody(),
         btVector3(0.0f, 0.0f, 0.0f),
         btVector3(0.0f, -FOOTBOT_WHEEL_Y_OFFSET, -FOOTBOT_WHEEL_HALF_DISTANCE),
         btVector3(0.0f, 1.0f, 0.0f),
         btVector3(0.0f, 0.0f, -1.0f));
      m_pcRightWheelToChassisConstraint = new btHingeConstraint(
         m_vecLocalBodies[Body::RIGHT_WHEEL]->GetRigidBody(),
         m_vecLocalBodies[Body::CHASSIS]->GetRigidBody(),
         btVector3(0.0f, 0.0f, 0.0f),
         btVector3(0.0f, -FOOTBOT_WHEEL_Y_OFFSET, FOOTBOT_WHEEL_HALF_DISTANCE),
         btVector3(0.0f, -1.0f, 0.0f),
         btVector3(0.0f, 0.0f, -1.0f));
      m_vecLocalConstraints.push_back(SConstraint("left-wheel:chassis",
                                                  m_pcLeftWheelToChassisConstraint,
                                                  true));
      m_vecLocalConstraints.push_back(SConstraint("right-wheel:chassis",
                                                  m_pcRightWheelToChassisConstraint,
                                                  true));
                                                                      
      /** Create the pivots **/
      btTransform pivotGeo(
         btQuaternion(0.0f, 0.0f, 0.0f, 1.0f),
         btVector3(0.0f, -FOOTBOT_PIVOT_RADIUS, 0.0f));

      m_vecLocalBodies.push_back(
         CDynamics3DBody::TNamedElement("front-pivot", new CDynamics3DBody(&m_cPivotCollisionShape,
                                                                           m_cFrontPivotTransform,
                                                                           pivotGeo,
                                                                           FOOTBOT_PIVOT_MASS)));
      m_vecLocalBodies.push_back(
         CDynamics3DBody::TNamedElement("rear-pivot", new CDynamics3DBody(&m_cPivotCollisionShape,
                                                                          m_cRearPivotTransform,
                                                                          pivotGeo,
                                                                          FOOTBOT_PIVOT_MASS)));

      /** create the pivots to base constraints **/
      m_pcFrontPivotToChassisConstraint = new btPoint2PointConstraint(
         m_vecLocalBodies[Body::FRONT_PIVOT]->GetRigidBody(),
         m_vecLocalBodies[Body::CHASSIS]->GetRigidBody(),
         btVector3(0.0f, 0.0f, 0.0f),
         btVector3(FOOTBOT_PIVOT_HALF_DISTANCE, -FOOTBOT_PIVOT_Y_OFFSET, 0.0f));
      m_pcRearPivotToChassisConstraint = new btPoint2PointConstraint(
         m_vecLocalBodies[Body::REAR_PIVOT]->GetRigidBody(),
         m_vecLocalBodies[Body::CHASSIS]->GetRigidBody(),
         btVector3(0.0f, 0.0f, 0.0f),
         btVector3(-FOOTBOT_PIVOT_HALF_DISTANCE, -FOOTBOT_PIVOT_Y_OFFSET, 0.0f));
      m_vecLocalConstraints.push_back(SConstraint("front-pivot:chassis",
                                                  m_pcFrontPivotToChassisConstraint,
                                                  true));
      m_vecLocalConstraints.push_back(SConstraint("rear-pivot:chassis",
                                                  m_pcRearPivotToChassisConstraint,
                                                  true));

      /** move the model to the specified coordinates */
      SetModelCoordinates(btTransform(ARGoSToBullet(GetEmbodiedEntity().GetInitOrientation()),
                                      ARGoSToBullet(GetEmbodiedEntity().GetInitPosition())));
   }

   /****************************************/
   /****************************************/

   CDynamics3DFootBotModel::~CDynamics3DFootBotModel() {
      delete m_pcLeftWheelToChassisConstraint;
      delete m_pcRightWheelToChassisConstraint;
      delete m_pcFrontPivotToChassisConstraint;
      delete m_pcRearPivotToChassisConstraint;
   }

   /****************************************/
   /****************************************/

   void CDynamics3DFootBotModel::UpdateEntityStatus() {
      /* Update footbot position and orientation based on the location of a reference body */
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
         
         m_pcLeftWheelToChassisConstraint->enableAngularMotor(true,
                                                              fLeftWheelVelocity,
                                                              FOOTBOT_WHEEL_MOTOR_IMPULSE);
         m_pcRightWheelToChassisConstraint->enableAngularMotor(true,
                                                               fRightWheelVelocity,
                                                               FOOTBOT_WHEEL_MOTOR_IMPULSE);
      }
      else {

         /* No, we don't want to move - zero all speeds */
         m_pcLeftWheelToChassisConstraint->enableAngularMotor(false, 0.0f, 0.0f);
         m_pcRightWheelToChassisConstraint->enableAngularMotor(false, 0.0f, 0.0f);
      }
   }

   /****************************************/
   /****************************************/

   btTransform CDynamics3DFootBotModel::GetModelCoordinates() const {
      return m_vecLocalBodies[Body::CHASSIS]->GetMotionStateTransform() *
         m_cChassisTransform.inverse();
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_DYNAMICS3D_OPERATIONS_ON_ENTITY(CFootBotEntity, CDynamics3DFootBotModel);

}
