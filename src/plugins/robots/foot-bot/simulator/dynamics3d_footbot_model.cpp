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

   btTransform CDynamics3DFootBotModel::m_cBodyTransform(
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

   // @todo This collision shape requires unthread safe init - see the constructor below!
   btCompoundShape CDynamics3DFootBotModel::m_cBodyCollisionShape;

   /****************************************/
   /****************************************/

   CDynamics3DFootBotModel::CDynamics3DFootBotModel(CDynamics3DEngine& c_engine,
                                                      CFootBotEntity& c_entity) :
      CDynamics3DModel(c_engine, c_entity.GetEmbodiedEntity()),
      m_cFootBotEntity(c_entity),
      m_cWheeledEntity(m_cFootBotEntity.GetWheeledEntity()) {

      if(m_cBodyCollisionShape.getNumChildShapes() == 0) {
         m_cBodyCollisionShape.addChildShape(m_cBatterySocketTransform, &m_cBatterySocketCollisionShape);
         m_cBodyCollisionShape.addChildShape(m_cBaseModuleTransform, &m_cBaseModuleCollisionShape);
      
         // DEBUG
         btVector3 minAabb;
         btVector3 maxAabb;
         m_cBodyCollisionShape.getAabb(btTransform::getIdentity(), minAabb, maxAabb);
         fprintf(stderr, "created footbot body collision shape with extents: [%.6f, %.6f, %.6f]\n", (maxAabb - minAabb).getX(), (maxAabb - minAabb).getY(), (maxAabb - minAabb).getZ());
      }

      // Vector for calculating interia
      btVector3 cInertia;

      // Transform representing the reference point and rotation of the footbot
      btTransform cModelTransform(ARGoSToBullet(GetEmbodiedEntity().GetOrientation()),
                                   ARGoSToBullet(GetEmbodiedEntity().GetPosition()));

      /** Create the body **/
      m_pcBodyMotionState = new btDefaultMotionState(cModelTransform * m_cBodyTransform, btTransform(
         btQuaternion(0.0f, 0.0f, 0.0f, 1.0f),
         btVector3(0.0f, -FOOTBOT_BATT_H, 0.0f)));
      m_cBodyCollisionShape.calculateLocalInertia(FOOTBOT_BASEMODULE_MASS, cInertia);
      m_pcBodyRigidBody = new btRigidBody(btRigidBody::btRigidBodyConstructionInfo(
         FOOTBOT_BASEMODULE_MASS, m_pcBodyMotionState, &m_cBodyCollisionShape, cInertia));
      m_mapLocalRigidBodies["body"] = m_pcBodyRigidBody;

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
      m_mapLocalRigidBodies["left-wheel"] = m_pcLeftWheelRigidBody;
      m_mapLocalRigidBodies["right-wheel"] = m_pcRightWheelRigidBody;

      /** create the wheels to body constraints **/
      m_pcLeftWheelToBodyConstraint = new btHingeConstraint(
         *m_pcLeftWheelRigidBody,
         *m_pcBodyRigidBody,
         btVector3(0.0f, 0.0f, 0.0f),
         btVector3(0.0f, -FOOTBOT_WHEEL_Y_OFFSET, -FOOTBOT_WHEEL_HALF_DISTANCE),
         btVector3(0.0f, 1.0f, 0.0f),
         btVector3(0.0f, 0.0f, -1.0f));
      m_pcRightWheelToBodyConstraint = new btHingeConstraint(
         *m_pcRightWheelRigidBody,
         *m_pcBodyRigidBody,
         btVector3(0.0f, 0.0f, 0.0f),
         btVector3(0.0f, -FOOTBOT_WHEEL_Y_OFFSET, FOOTBOT_WHEEL_HALF_DISTANCE),
         btVector3(0.0f, -1.0f, 0.0f),
         btVector3(0.0f, 0.0f, -1.0f));
      m_mapLocalConstraints["left-wheel:body"] = m_pcLeftWheelToBodyConstraint;
      m_mapLocalConstraints["right-wheel:body"] = m_pcRightWheelToBodyConstraint;

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
      m_mapLocalRigidBodies["front-pivot"] = m_pcFrontPivotRigidBody;
      m_mapLocalRigidBodies["rear-pivot"] = m_pcRearPivotRigidBody;

      /** create the pivots to base constraints **/
      m_pcFrontPivotToBodyConstraint = new btPoint2PointConstraint(
         *m_pcFrontPivotRigidBody,
         *m_pcBodyRigidBody,
         btVector3(0.0f, 0.0f, 0.0f),
         btVector3(FOOTBOT_PIVOT_HALF_DISTANCE, -FOOTBOT_PIVOT_Y_OFFSET, 0.0f));
      m_pcRearPivotToBodyConstraint = new btPoint2PointConstraint(
         *m_pcRearPivotRigidBody,
         *m_pcBodyRigidBody,
         btVector3(0.0f, 0.0f, 0.0f),
         btVector3(-FOOTBOT_PIVOT_HALF_DISTANCE, -FOOTBOT_PIVOT_Y_OFFSET, 0.0f));
      m_mapLocalConstraints["front-pivot:body"] = m_pcFrontPivotToBodyConstraint;
      m_mapLocalConstraints["rear-pivot:body"] = m_pcRearPivotToBodyConstraint;
   }

   /****************************************/
   /****************************************/

   CDynamics3DFootBotModel::~CDynamics3DFootBotModel() {
      delete m_pcLeftWheelToBodyConstraint;
      delete m_pcRightWheelToBodyConstraint;
      delete m_pcFrontPivotToBodyConstraint;
      delete m_pcRearPivotToBodyConstraint;
      delete m_pcLeftWheelMotionState;
      delete m_pcRightWheelMotionState;
      delete m_pcFrontPivotMotionState;
      delete m_pcRearPivotMotionState;
      delete m_pcLeftWheelRigidBody;
      delete m_pcRightWheelRigidBody;
      delete m_pcFrontPivotRigidBody;
      delete m_pcRearPivotRigidBody;
      delete m_pcBodyRigidBody;
   }

   /****************************************/
   /****************************************/

   bool CDynamics3DFootBotModel::CheckIntersectionWithRay(Real& f_t_on_ray,
                                                           const CRay3& c_ray) const {
      return false;
   }

   /****************************************/
   /****************************************/

   bool CDynamics3DFootBotModel::MoveTo(const CVector3& c_position,
                                         const CQuaternion& c_orientation,
                                         bool b_check_only) {
      return false;
   }

   /****************************************/
   /****************************************/

   void CDynamics3DFootBotModel::UpdateEntityStatus() {
      /* Update footbot position and orientation */
      const btTransform& cUpdateTransform = m_pcBodyMotionState->m_graphicsWorldTrans * m_cBodyTransform.inverse();

      GetEmbodiedEntity().SetPosition(BulletToARGoS(cUpdateTransform.getOrigin()));
      GetEmbodiedEntity().SetOrientation(BulletToARGoS(cUpdateTransform.getRotation()));

      //fprintf(stderr, "position of %s in ARGoS: [%.3f, %.3f, %.3f]\n", m_cFootBotEntity.GetId().c_str(), GetEmbodiedEntity().GetPosition().GetX(), GetEmbodiedEntity().GetPosition().GetY(),GetEmbodiedEntity().GetPosition().GetZ());

      /* Update components */
      m_cFootBotEntity.UpdateComponents();
   }

   /****************************************/
   /****************************************/

   

   void CDynamics3DFootBotModel::UpdateFromEntityStatus() {

      for(std::map<std::string, btRigidBody*>::iterator it = m_mapLocalRigidBodies.begin();
          it != m_mapLocalRigidBodies.end();
          it++) {
         
         btDefaultMotionState* pcMotionState = dynamic_cast<btDefaultMotionState*>(it->second->getMotionState());

         fprintf(stderr, "[DEBUG] %s/m_graphicsWorldTrans: position = [%.3f, %.3f, %.3f], rotation axis = [%.3f, %.3f, %.3f] & angle = %.3f\n", it->first.c_str(), pcMotionState->m_graphicsWorldTrans.getOrigin().getX(), pcMotionState->m_graphicsWorldTrans.getOrigin().getY(), pcMotionState->m_graphicsWorldTrans.getOrigin().getZ(), pcMotionState->m_graphicsWorldTrans.getRotation().getAxis().getX(), pcMotionState->m_graphicsWorldTrans.getRotation().getAxis().getY(), pcMotionState->m_graphicsWorldTrans.getRotation().getAxis().getZ(), pcMotionState->m_graphicsWorldTrans.getRotation().getAngle() * 57.2957795131f);

         fprintf(stderr, "[DEBUG] %s/m_centerOfMassOffset: position = [%.3f, %.3f, %.3f], rotation axis = [%.3f, %.3f, %.3f] & angle = %.3f\n", it->first.c_str(), pcMotionState->m_centerOfMassOffset.getOrigin().getX(), pcMotionState->m_centerOfMassOffset.getOrigin().getY(), pcMotionState->m_centerOfMassOffset.getOrigin().getZ(), pcMotionState->m_centerOfMassOffset.getRotation().getAxis().getX(), pcMotionState->m_centerOfMassOffset.getRotation().getAxis().getY(), pcMotionState->m_centerOfMassOffset.getRotation().getAxis().getZ(), pcMotionState->m_centerOfMassOffset.getRotation().getAngle() * 57.2957795131f);

         fprintf(stderr, "[DEBUG] %s/m_startWorldTrans:    position = [%.3f, %.3f, %.3f], rotation axis = [%.3f, %.3f, %.3f] & angle = %.3f\n", it->first.c_str(), pcMotionState->m_startWorldTrans.getOrigin().getX(), pcMotionState->m_startWorldTrans.getOrigin().getY(), pcMotionState->m_startWorldTrans.getOrigin().getZ(), pcMotionState->m_startWorldTrans.getRotation().getAxis().getX(), pcMotionState->m_startWorldTrans.getRotation().getAxis().getY(), pcMotionState->m_startWorldTrans.getRotation().getAxis().getZ(), pcMotionState->m_startWorldTrans.getRotation().getAngle() * 57.2957795131f);
      }
         

      /* Get wheel speeds from entity */
      const Real* m_pfCurrentWheelVelocityFromSensor = m_cWheeledEntity.GetWheelVelocities();

      /* Do we want to move? */
      if((m_pfCurrentWheelVelocityFromSensor[FOOTBOT_LEFT_WHEEL] != 0.0f) ||
         (m_pfCurrentWheelVelocityFromSensor[FOOTBOT_RIGHT_WHEEL] != 0.0f)) {

         Real fLeftWheelVelocity, fRightWheelVelocity;

         fLeftWheelVelocity  = m_pfCurrentWheelVelocityFromSensor[FOOTBOT_LEFT_WHEEL] / FOOTBOT_WHEEL_RADIUS;
         fRightWheelVelocity = m_pfCurrentWheelVelocityFromSensor[FOOTBOT_RIGHT_WHEEL] / FOOTBOT_WHEEL_RADIUS;

         // activate the bodies! note: it might be possible just to activate the wheels... alternatively we should just use the pointers here to avoid iterating over the map during the update loop
         for(std::map<std::string, btRigidBody*>::iterator itBody = m_mapLocalRigidBodies.begin();
             itBody !=  m_mapLocalRigidBodies.end();
             itBody++) {
            itBody->second->activate();
         }

         m_pcLeftWheelToBodyConstraint->enableAngularMotor(true, fLeftWheelVelocity, FOOTBOT_WHEEL_MOTOR_IMPULSE);
         m_pcRightWheelToBodyConstraint->enableAngularMotor(true, fRightWheelVelocity, FOOTBOT_WHEEL_MOTOR_IMPULSE);
      }
      else {

         /* No, we don't want to move - zero all speeds */
         m_pcLeftWheelToBodyConstraint->enableAngularMotor(false, 0.0f, 0.0f);
         m_pcRightWheelToBodyConstraint->enableAngularMotor(false, 0.0f, 0.0f);
      }
      //fprintf(stderr, "[DEBUG] lwheel hinge angle: %.3f\n", m_pcLeftWheelToBodyConstraint->getHingeAngle() );
      //fprintf(stderr, "[DEBUG] rwheel hinge angle: %.3f\n", m_pcRightWheelToBodyConstraint->getHingeAngle() );

   }

   /****************************************/
   /****************************************/

   void CDynamics3DFootBotModel::CalculateBoundingBox() {
      /** @todo Calculate foot-bot bounding box */
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_DYNAMICS3D_OPERATIONS_ON_ENTITY(CFootBotEntity, CDynamics3DFootBotModel);

}
