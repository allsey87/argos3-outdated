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
      btQuaternion(btVector3(1.0f, 0.0f, 0.0f), -ARGOS_PI * 0.5f),
      btVector3(0.0f, FOOTBOT_WHEEL_RADIUS, FOOTBOT_WHEEL_HALF_DISTANCE + FOOTBOT_WHEEL_THICKNESS * 0.5f));

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
   m_pcLeftWheelToChassisConstraint = new btGeneric6DofConstraint(
         m_vecLocalBodies[Body::LEFT_WHEEL]->GetRigidBody(),
         m_vecLocalBodies[Body::CHASSIS]->GetRigidBody(),
         btTransform(
                     btQuaternion(0.0f, 0.0f, 0.0f, 1.0f),
                     btVector3(0.0f, 0.0f, 0.0f)),
         btTransform(
                     btQuaternion(btVector3(1.0f, 0.0f, 0.0f), -ARGOS_PI * 0.5f),
                     btVector3(0.0f, -FOOTBOT_WHEEL_Y_OFFSET, -FOOTBOT_WHEEL_HALF_DISTANCE)),
         true);

      // setup constraint limits, all DOF locked except rotation around the frame's Y axis which is free
      m_pcLeftWheelToChassisConstraint->setLinearLowerLimit(btVector3(0,0,0));
      m_pcLeftWheelToChassisConstraint->setLinearUpperLimit(btVector3(0,0,0));
      m_pcLeftWheelToChassisConstraint->setAngularLowerLimit(btVector3(0,1,0));
      m_pcLeftWheelToChassisConstraint->setAngularUpperLimit(btVector3(0,-1,0));

      m_pcRightWheelToChassisConstraint = new btGeneric6DofConstraint(
         m_vecLocalBodies[Body::RIGHT_WHEEL]->GetRigidBody(),
         m_vecLocalBodies[Body::CHASSIS]->GetRigidBody(),
         btTransform(
                     btQuaternion(0.0f, 0.0f, 0.0f, 1.0f),
                     btVector3(0.0f, 0.0f, 0.0f)),
         btTransform(
                     btQuaternion(btVector3(1.0f, 0.0f, 0.0f), -ARGOS_PI * 0.5f),
                     btVector3(0.0f, -FOOTBOT_WHEEL_Y_OFFSET, FOOTBOT_WHEEL_HALF_DISTANCE)),
         true);
      
      // setup constraint limits, all DOF locked except rotation around the frame's Y axis which is free
      m_pcRightWheelToChassisConstraint->setLinearLowerLimit(btVector3(0,0,0));
      m_pcRightWheelToChassisConstraint->setLinearUpperLimit(btVector3(0,0,0));
      m_pcRightWheelToChassisConstraint->setAngularLowerLimit(btVector3(0,1,0));
      m_pcRightWheelToChassisConstraint->setAngularUpperLimit(btVector3(0,-1,0));


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
      m_pcFrontPivotToChassisConstraint = new btGeneric6DofConstraint(
         m_vecLocalBodies[Body::FRONT_PIVOT]->GetRigidBody(),
         m_vecLocalBodies[Body::CHASSIS]->GetRigidBody(),
         btTransform(
                     btQuaternion(0.0f, 0.0f, 0.0f, 1.0f),
                     btVector3(0.0f, 0.0f, 0.0f)),
         btTransform(
                     btQuaternion(0.0f, 0.0f, 0.0f, 1.0f),
                     btVector3(FOOTBOT_PIVOT_HALF_DISTANCE, -FOOTBOT_PIVOT_Y_OFFSET, 0.0f)),
         true);

      // Lock the translational DOFs and free the rotational DOFs
      m_pcFrontPivotToChassisConstraint->setLinearLowerLimit(btVector3(0,0,0));
      m_pcFrontPivotToChassisConstraint->setLinearUpperLimit(btVector3(0,0,0));
      m_pcFrontPivotToChassisConstraint->setAngularLowerLimit(btVector3(1,1,1));
      m_pcFrontPivotToChassisConstraint->setAngularUpperLimit(btVector3(-1,-1,-1));


      m_pcRearPivotToChassisConstraint = new btGeneric6DofConstraint(
         m_vecLocalBodies[Body::REAR_PIVOT]->GetRigidBody(),
         m_vecLocalBodies[Body::CHASSIS]->GetRigidBody(),
         btTransform(
                     btQuaternion(0.0f, 0.0f, 0.0f, 1.0f),
                     btVector3(0.0f, 0.0f, 0.0f)),
         btTransform(
                     btQuaternion(0.0f, 0.0f, 0.0f, 1.0f),
                     btVector3(-FOOTBOT_PIVOT_HALF_DISTANCE, -FOOTBOT_PIVOT_Y_OFFSET, 0.0f)),
         true);

      // Lock the translational DOFs and free the rotational DOFs      
      m_pcRearPivotToChassisConstraint->setLinearLowerLimit(btVector3(0,0,0));
      m_pcRearPivotToChassisConstraint->setLinearUpperLimit(btVector3(0,0,0));
      m_pcRearPivotToChassisConstraint->setAngularLowerLimit(btVector3(1,1,1));
      m_pcRearPivotToChassisConstraint->setAngularUpperLimit(btVector3(-1,-1,-1));

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
      /*
      for(CDynamics3DBody::TNamedVector::iterator it = m_vecLocalBodies.begin();
          it != m_vecLocalBodies.end();
          it++) {

         btDefaultMotionState* pcMotionState = &it->second->GetMotionState();

         fprintf(stderr, "[DEBUG] %s/m_graphicsWorldTrans: position = [%.3f, %.3f, %.3f], rotation axis = [%.3f, %.3f, %.3f] & angle = %.3f\n", it->first.c_str(), pcMotionState->m_graphicsWorldTrans.getOrigin().getX(), pcMotionState->m_graphicsWorldTrans.getOrigin().getY(), pcMotionState->m_graphicsWorldTrans.getOrigin().getZ(), pcMotionState->m_graphicsWorldTrans.getRotation().getAxis().getX(), pcMotionState->m_graphicsWorldTrans.getRotation().getAxis().getY(), pcMotionState->m_graphicsWorldTrans.getRotation().getAxis().getZ(), pcMotionState->m_graphicsWorldTrans.getRotation().getAngle() * 57.2957795131f);

         }
      */
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
         
         m_pcLeftWheelToChassisConstraint->getRotationalLimitMotor(1)->m_enableMotor = true;
         m_pcLeftWheelToChassisConstraint->getRotationalLimitMotor(1)->m_targetVelocity = fLeftWheelVelocity;
         m_pcLeftWheelToChassisConstraint->getRotationalLimitMotor(1)->m_maxMotorForce = FOOTBOT_WHEEL_MOTOR_IMPULSE;

         m_pcRightWheelToChassisConstraint->getRotationalLimitMotor(1)->m_enableMotor = true;
         m_pcRightWheelToChassisConstraint->getRotationalLimitMotor(1)->m_targetVelocity = fRightWheelVelocity;
         m_pcRightWheelToChassisConstraint->getRotationalLimitMotor(1)->m_maxMotorForce = FOOTBOT_WHEEL_MOTOR_IMPULSE;

      }
      else {

         /* No, we don't want to move - zero all speeds */
         m_pcLeftWheelToChassisConstraint->getRotationalLimitMotor(1)->m_enableMotor = false;
         m_pcRightWheelToChassisConstraint->getRotationalLimitMotor(1)->m_enableMotor = false;
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
