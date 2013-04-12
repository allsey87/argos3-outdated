/**
 * @file <argos3/plugins/robots/foot-bot/simulator/dynamics3d_footbot_model.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "dynamics3d_footbot_model.h"
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_engine.h>

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
      
         /* //DEBUG
         btVector3 minAabb;
         btVector3 maxAabb;
         m_cBodyCollisionShape.getAabb(btTransform::getIdentity(), minAabb, maxAabb);
         fprintf(stderr, "created footbot body collision shape with extents: [%.6f, %.6f, %.6f]\n", (maxAabb - minAabb).getX(), (maxAabb - minAabb).getY(), (maxAabb - minAabb).getZ()); */
      }

      // Vector for calculating interia
      btVector3 cInertia;

      // Transform representing the reference point and rotation of the footbot
      btTransform cModelTransform(ARGoSToBullet(GetEmbodiedEntity().GetInitOrientation()),
                                  ARGoSToBullet(GetEmbodiedEntity().GetInitPosition()));

      /** Create the body **/
      m_pcChassisMotionState = new btDefaultMotionState(cModelTransform * m_cChassisTransform, btTransform(
         btQuaternion(0.0f, 0.0f, 0.0f, 1.0f),
         btVector3(0.0f, -FOOTBOT_BATT_H, 0.0f)));
      m_cChassisCollisionShape.calculateLocalInertia(FOOTBOT_BASEMODULE_MASS, cInertia);
      m_pcChassisRigidBody = new btRigidBody(btRigidBody::btRigidBodyConstructionInfo(
         FOOTBOT_BASEMODULE_MASS, m_pcChassisMotionState, &m_cChassisCollisionShape, cInertia));
      m_mapLocalBodyConfigurations["chassis"] = SBodyConfiguration(&m_cChassisCollisionShape,
                                                                   m_pcChassisMotionState,
                                                                   m_pcChassisRigidBody);

      /** create the wheels **/
      m_pcLeftWheelMotionState = new btDefaultMotionState(cModelTransform * m_cLeftWheelTransform, btTransform(
         btQuaternion(0.0f, 0.0f, 0.0f, 1.0f),
         btVector3(0.0f, -FOOTBOT_WHEEL_THICKNESS * 0.5f, 0.0f)));
      m_pcRightWheelMotionState = new btDefaultMotionState(cModelTransform * m_cRightWheelTransform, btTransform(
         btQuaternion(0.0f, 0.0f, 0.0f, 1.0f),
         btVector3(0.0f, -FOOTBOT_WHEEL_THICKNESS * 0.5f, 0.0f)));
      m_cWheelCollisionShape.calculateLocalInertia(FOOTBOT_WHEEL_MASS, cInertia);
      m_pcLeftWheelRigidBody = new btRigidBody(btRigidBody::btRigidBodyConstructionInfo(
         FOOTBOT_WHEEL_MASS, m_pcLeftWheelMotionState, &m_cWheelCollisionShape, cInertia));
      m_pcRightWheelRigidBody = new btRigidBody(btRigidBody::btRigidBodyConstructionInfo(
         FOOTBOT_WHEEL_MASS, m_pcRightWheelMotionState, &m_cWheelCollisionShape, cInertia));
      m_mapLocalBodyConfigurations["left-wheel"] = SBodyConfiguration(&m_cWheelCollisionShape,
                                                                      m_pcLeftWheelMotionState,
                                                                      m_pcLeftWheelRigidBody);
      m_mapLocalBodyConfigurations["right-wheel"] = SBodyConfiguration(&m_cWheelCollisionShape,
                                                                       m_pcRightWheelMotionState,
                                                                       m_pcRightWheelRigidBody);

      /** create the wheels to body constraints **/
      m_pcLeftWheelToChassisConstraint = new btHingeConstraint(
         *m_pcLeftWheelRigidBody,
         *m_pcChassisRigidBody,
         btVector3(0.0f, 0.0f, 0.0f),
         btVector3(0.0f, -FOOTBOT_WHEEL_Y_OFFSET, -FOOTBOT_WHEEL_HALF_DISTANCE),
         btVector3(0.0f, 1.0f, 0.0f),
         btVector3(0.0f, 0.0f, -1.0f));
      m_pcRightWheelToChassisConstraint = new btHingeConstraint(
         *m_pcRightWheelRigidBody,
         *m_pcChassisRigidBody,
         btVector3(0.0f, 0.0f, 0.0f),
         btVector3(0.0f, -FOOTBOT_WHEEL_Y_OFFSET, FOOTBOT_WHEEL_HALF_DISTANCE),
         btVector3(0.0f, -1.0f, 0.0f),
         btVector3(0.0f, 0.0f, -1.0f));
      m_mapLocalConstraints["left-wheel:chassis"] = SConstraint(m_pcLeftWheelToChassisConstraint, true);
      m_mapLocalConstraints["right-wheel:chassis"] = SConstraint(m_pcRightWheelToChassisConstraint, true);

      /** Create the pivots **/
      m_pcFrontPivotMotionState = new btDefaultMotionState(cModelTransform * m_cFrontPivotTransform, btTransform(
         btQuaternion(0.0f, 0.0f, 0.0f, 1.0f),
         btVector3(0.0f, -FOOTBOT_PIVOT_RADIUS, 0.0f)));
      m_pcRearPivotMotionState = new btDefaultMotionState(cModelTransform * m_cRearPivotTransform, btTransform(
         btQuaternion(0.0f, 0.0f, 0.0f, 1.0f),
         btVector3(0.0f, -FOOTBOT_PIVOT_RADIUS, 0.0f)));
      m_cPivotCollisionShape.calculateLocalInertia(FOOTBOT_PIVOT_MASS, cInertia);
      m_pcFrontPivotRigidBody = new btRigidBody(btRigidBody::btRigidBodyConstructionInfo(
         FOOTBOT_PIVOT_MASS, m_pcFrontPivotMotionState, &m_cPivotCollisionShape, cInertia));
      m_pcRearPivotRigidBody = new btRigidBody(btRigidBody::btRigidBodyConstructionInfo(
         FOOTBOT_PIVOT_MASS, m_pcRearPivotMotionState, &m_cPivotCollisionShape, cInertia));
      m_mapLocalBodyConfigurations["front-pivot"] = SBodyConfiguration(&m_cPivotCollisionShape,
                                                                       m_pcFrontPivotMotionState,
                                                                       m_pcFrontPivotRigidBody);
      m_mapLocalBodyConfigurations["rear-pivot"] = SBodyConfiguration(&m_cPivotCollisionShape,
                                                                      m_pcRearPivotMotionState,
                                                                      m_pcRearPivotRigidBody);

      /** create the pivots to base constraints **/
      m_pcFrontPivotToChassisConstraint = new btPoint2PointConstraint(
         *m_pcFrontPivotRigidBody,
         *m_pcChassisRigidBody,
         btVector3(0.0f, 0.0f, 0.0f),
         btVector3(FOOTBOT_PIVOT_HALF_DISTANCE, -FOOTBOT_PIVOT_Y_OFFSET, 0.0f));
      m_pcRearPivotToChassisConstraint = new btPoint2PointConstraint(
         *m_pcRearPivotRigidBody,
         *m_pcChassisRigidBody,
         btVector3(0.0f, 0.0f, 0.0f),
         btVector3(-FOOTBOT_PIVOT_HALF_DISTANCE, -FOOTBOT_PIVOT_Y_OFFSET, 0.0f));
      m_mapLocalConstraints["front-pivot:chassis"] = SConstraint(m_pcFrontPivotToChassisConstraint, true);
      m_mapLocalConstraints["rear-pivot:chassis"] = SConstraint(m_pcRearPivotToChassisConstraint, true); 
   }

   /****************************************/
   /****************************************/

   CDynamics3DFootBotModel::~CDynamics3DFootBotModel() {
      delete m_pcLeftWheelToChassisConstraint;
      delete m_pcRightWheelToChassisConstraint;
      delete m_pcFrontPivotToChassisConstraint;
      delete m_pcRearPivotToChassisConstraint;
      
      delete m_pcLeftWheelMotionState;
      delete m_pcRightWheelMotionState;
      delete m_pcFrontPivotMotionState;
      delete m_pcRearPivotMotionState;
      delete m_pcChassisMotionState;
      
      delete m_pcLeftWheelRigidBody;
      delete m_pcRightWheelRigidBody;
      delete m_pcFrontPivotRigidBody;
      delete m_pcRearPivotRigidBody;
      delete m_pcChassisRigidBody;
   }

   /****************************************/
   /****************************************/

   void CDynamics3DFootBotModel::UpdateEntityStatus() {
      /* Update footbot position and orientation based on the location of a reference body */
      const btTransform& cUpdateTransform = m_pcChassisMotionState->m_graphicsWorldTrans * m_cChassisTransform.inverse();

      GetEmbodiedEntity().SetPosition(BulletToARGoS(cUpdateTransform.getOrigin()));
      GetEmbodiedEntity().SetOrientation(BulletToARGoS(cUpdateTransform.getRotation()));

      //fprintf(stderr, "position of %s in ARGoS: [%.3f, %.3f, %.3f]\n", m_cFootBotEntity.GetId().c_str(), GetEmbodiedEntity().GetPosition().GetX(), GetEmbodiedEntity().GetPosition().GetY(),GetEmbodiedEntity().GetPosition().GetZ());

      /* Update components */
      m_cFootBotEntity.UpdateComponents();
   }

   /****************************************/
   /****************************************/

   

   void CDynamics3DFootBotModel::UpdateFromEntityStatus() {
      /*
      for(std::map<std::string, btRigidBody*>::iterator it = m_mapLocalRigidBodies.begin();
          it != m_mapLocalRigidBodies.end();
          it++) {
         
         btDefaultMotionState* pcMotionState = dynamic_cast<btDefaultMotionState*>(it->second->getMotionState());

         fprintf(stderr, "[DEBUG] %s/m_graphicsWorldTrans: position = [%.3f, %.3f, %.3f], rotation axis = [%.3f, %.3f, %.3f] & angle = %.3f\n", it->first.c_str(), pcMotionState->m_graphicsWorldTrans.getOrigin().getX(), pcMotionState->m_graphicsWorldTrans.getOrigin().getY(), pcMotionState->m_graphicsWorldTrans.getOrigin().getZ(), pcMotionState->m_graphicsWorldTrans.getRotation().getAxis().getX(), pcMotionState->m_graphicsWorldTrans.getRotation().getAxis().getY(), pcMotionState->m_graphicsWorldTrans.getRotation().getAxis().getZ(), pcMotionState->m_graphicsWorldTrans.getRotation().getAngle() * 57.2957795131f);

         fprintf(stderr, "[DEBUG] %s/m_centerOfMassOffset: position = [%.3f, %.3f, %.3f], rotation axis = [%.3f, %.3f, %.3f] & angle = %.3f\n", it->first.c_str(), pcMotionState->m_centerOfMassOffset.getOrigin().getX(), pcMotionState->m_centerOfMassOffset.getOrigin().getY(), pcMotionState->m_centerOfMassOffset.getOrigin().getZ(), pcMotionState->m_centerOfMassOffset.getRotation().getAxis().getX(), pcMotionState->m_centerOfMassOffset.getRotation().getAxis().getY(), pcMotionState->m_centerOfMassOffset.getRotation().getAxis().getZ(), pcMotionState->m_centerOfMassOffset.getRotation().getAngle() * 57.2957795131f);

         fprintf(stderr, "[DEBUG] %s/m_startWorldTrans:    position = [%.3f, %.3f, %.3f], rotation axis = [%.3f, %.3f, %.3f] & angle = %.3f\n", it->first.c_str(), pcMotionState->m_startWorldTrans.getOrigin().getX(), pcMotionState->m_startWorldTrans.getOrigin().getY(), pcMotionState->m_startWorldTrans.getOrigin().getZ(), pcMotionState->m_startWorldTrans.getRotation().getAxis().getX(), pcMotionState->m_startWorldTrans.getRotation().getAxis().getY(), pcMotionState->m_startWorldTrans.getRotation().getAxis().getZ(), pcMotionState->m_startWorldTrans.getRotation().getAngle() * 57.2957795131f);
      }
      */ 

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
         m_pcChassisRigidBody->activate();
         
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
      //fprintf(stderr, "[DEBUG] lwheel hinge angle: %.3f\n", m_pcLeftWheelToBodyConstraint->getHingeAngle() );
      //fprintf(stderr, "[DEBUG] rwheel hinge angle: %.3f\n", m_pcRightWheelToBodyConstraint->getHingeAngle() );

   }

   /****************************************/
   /****************************************/

   //void CDynamics3DFootBotModel::CalculateBoundingBox() {
      /** @todo Calculate foot-bot bounding box */
   //}

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_DYNAMICS3D_OPERATIONS_ON_ENTITY(CFootBotEntity, CDynamics3DFootBotModel);

}
