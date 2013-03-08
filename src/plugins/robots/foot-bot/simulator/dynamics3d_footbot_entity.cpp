/**
 * @file <argos3/plugins/robots/foot-bot/simulator/dynamics3d_footbot_entity.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "dynamics3d_footbot_entity.h"
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_engine.h>

namespace argos {

   static const Real FOOTBOT_WHEEL_RADIUS          =  0.029112741f;
   static const Real FOOTBOT_WHEEL_THICKNESS       =  0.022031354f;
   static const Real FOOTBOT_WHEEL_X_OFFSET        =  0.070000000f;
   static const Real FOOTBOT_WHEEL_MASS            =  0.100000000f;
   static const Real FOOTBOT_WHEEL_MOTOR_IMPULSE   =  1.500000000f;
   
   static const Real FOOTBOT_BATT_W                =  0.081662841f;
   static const Real FOOTBOT_BATT_H                =  FOOTBOT_WHEEL_RADIUS * 2;
   static const Real FOOTBOT_BATT_L                =  0.150302467f;
   static const Real FOOTBOT_BATT_GROUND_OFFSET    =  0.006000000f;
   static const Real FOOTBOT_BODY_RADIUS           =  0.085036758f;
   static const Real FOOTBOT_BODY_HEIGHT           =  0.072000000f;
   static const Real FOOTBOT_BODY_Y_OFFSET         =  (FOOTBOT_BATT_H + FOOTBOT_BODY_HEIGHT) / 2.0f;
   static const Real FOOTBOT_BODY_MASS             =  0.100000000f;
   
   static const Real FOOTBOT_PIVOT_RADIUS          =  FOOTBOT_WHEEL_RADIUS;
   static const Real FOOTBOT_PIVOT_Z_OFFSET        =  0.050000000f;
   static const Real FOOTBOT_PIVOT_MASS            =  0.100000000f;
   
   enum EFootbotWheels {
      FOOTBOT_LEFT_WHEEL  = 0,
      FOOTBOT_RIGHT_WHEEL = 1
   };
   
   /****************************************/
   /****************************************/
   
   // Shared static transforms
   
   btTransform CDynamics3DFootBotEntity::m_cLeftWheelTransform(
      btQuaternion(btVector3(0.0f,0.0f,1.0f), 1.57f), 
      btVector3(-FOOTBOT_WHEEL_X_OFFSET, FOOTBOT_WHEEL_RADIUS, 0.0f));
   btTransform CDynamics3DFootBotEntity::m_cRightWheelTransform(
      btQuaternion(btVector3(0.0f,0.0f,1.0f), -1.57f), 
      btVector3(FOOTBOT_WHEEL_X_OFFSET, FOOTBOT_WHEEL_RADIUS, 0.0f));
   btTransform CDynamics3DFootBotEntity::m_cFrontPivotTransform(
      btQuaternion(0.0f,0.0f,0.0f, 1.0f),
      btVector3(0.0f, FOOTBOT_PIVOT_RADIUS, FOOTBOT_PIVOT_Z_OFFSET));
   btTransform CDynamics3DFootBotEntity::m_cRearPivotTransform(
      btQuaternion(0.0f,0.0f,0.0f, 1.0f),
      btVector3(0.0f, FOOTBOT_PIVOT_RADIUS, -FOOTBOT_PIVOT_Z_OFFSET));
      
   btTransform CDynamics3DFootBotEntity::m_cUpperBodyTransform(
      btQuaternion(0.0f,0.0f,0.0f, 1.0f), 
      btVector3(0.0f, FOOTBOT_BODY_HEIGHT / 2.0f, 0.0f));
   btTransform CDynamics3DFootBotEntity::m_cLowerBodyTransform(
      btQuaternion(0.0f,0.0f,0.0f, 1.0f), 
      btVector3(0.0f, -FOOTBOT_BATT_H / 2.0f, 0.0f));
   btTransform CDynamics3DFootBotEntity::m_cBodyTransform(
      btQuaternion(0.0f,0.0f,0.0f, 1.0f), 
      btVector3(0.0f, FOOTBOT_BATT_GROUND_OFFSET + FOOTBOT_BATT_H, 0.0f));

   // Shared static collision shapes

   btBoxShape CDynamics3DFootBotEntity::m_cLowerBodyCollisionShape(
      btVector3(FOOTBOT_BATT_W, FOOTBOT_BATT_H, FOOTBOT_BATT_L) / 2.0f);
   btCylinderShape CDynamics3DFootBotEntity::m_cUpperBodyCollisionShape(
      btVector3(FOOTBOT_BODY_RADIUS, FOOTBOT_BODY_HEIGHT / 2.0f, FOOTBOT_BODY_RADIUS));
   btCylinderShape CDynamics3DFootBotEntity::m_cWheelCollisionShape(
      btVector3(FOOTBOT_WHEEL_RADIUS, FOOTBOT_WHEEL_THICKNESS * 0.5f, FOOTBOT_WHEEL_RADIUS));
   btSphereShape CDynamics3DFootBotEntity::m_cPivotCollisionShape(FOOTBOT_PIVOT_RADIUS);
   
   // TODO This collision shape requires unthread safe init - see the constructor below!
   btCompoundShape CDynamics3DFootBotEntity::m_cBodyCollisionShape;
   
   /****************************************/
   /****************************************/

   CDynamics3DFootBotEntity::CDynamics3DFootBotEntity(CDynamics3DEngine& c_engine,
                                                      CFootBotEntity& c_entity) :
      CDynamics3DEntity(c_engine, c_entity.GetEmbodiedEntity()),
      m_cFootBotEntity(c_entity),
      m_cWheeledEntity(m_cFootBotEntity.GetWheeledEntity()) {
      
      /* Unthread safe init of compound shape to be fixed */
      if(m_cBodyCollisionShape.getNumChildShapes() == 0) {
         m_cBodyCollisionShape.addChildShape(m_cLowerBodyTransform, &m_cLowerBodyCollisionShape);
         m_cBodyCollisionShape.addChildShape(m_cUpperBodyTransform, &m_cUpperBodyCollisionShape);
      }
      
     
      // Vector for calculating interia   
      btVector3 cInteria;
      
      // Transform representing the reference point and rotation of the footbot
      btTransform cEntityTransform(ARGoSToBullet(GetEmbodiedEntity().GetOrientation()),
                                   ARGoSToBullet(GetEmbodiedEntity().GetPosition()));
      
      
      /** Create the body **/
      m_pcBodyMotionState = new btDefaultMotionState(cEntityTransform * m_cBodyTransform);
      m_cBodyCollisionShape.calculateLocalInertia(FOOTBOT_BODY_MASS, cInteria);
      m_pcBodyRigidBody = new btRigidBody(btRigidBody::btRigidBodyConstructionInfo(
         FOOTBOT_BODY_MASS, m_pcBodyMotionState, &m_cBodyCollisionShape, cInteria));
      
      m_vecLocalRigidBodies.push_back(m_pcBodyRigidBody);
      
      /** create the wheels **/
      m_pcLeftWheelMotionState = new btDefaultMotionState(cEntityTransform * m_cLeftWheelTransform);
      m_pcRightWheelMotionState = new btDefaultMotionState(cEntityTransform * m_cRightWheelTransform);
      m_cWheelCollisionShape.calculateLocalInertia(FOOTBOT_WHEEL_MASS, cInteria);
      m_pcLeftWheelRigidBody = new btRigidBody(btRigidBody::btRigidBodyConstructionInfo(
         FOOTBOT_WHEEL_MASS, m_pcLeftWheelMotionState, &m_cWheelCollisionShape, cInteria));
      m_pcRightWheelRigidBody = new btRigidBody(btRigidBody::btRigidBodyConstructionInfo(
         FOOTBOT_WHEEL_MASS, m_pcRightWheelMotionState, &m_cWheelCollisionShape, cInteria));
      
      m_vecLocalRigidBodies.push_back(m_pcLeftWheelRigidBody);
      m_vecLocalRigidBodies.push_back(m_pcRightWheelRigidBody);
      
      /** create the wheels to base constraints **/
      m_pcLeftWheelToBodyConstraint = new btHingeConstraint(
         *m_pcLeftWheelRigidBody,
         *m_pcBodyRigidBody,
         btVector3(0.0f, 0.0f, 0.0f),
         btVector3(-FOOTBOT_WHEEL_X_OFFSET, -FOOTBOT_BATT_GROUND_OFFSET - FOOTBOT_BATT_H / 2.0f, 0.0f),
         btVector3(0.0f, 1.0f, 0.0f), 
         btVector3(-1.0f, 0.0f, 0.0f));
      m_pcRightWheelToBodyConstraint = new btHingeConstraint(
         *m_pcRightWheelRigidBody, 
         *m_pcBodyRigidBody,
         btVector3(0.0f, 0.0f, 0.0f),
         btVector3(FOOTBOT_WHEEL_X_OFFSET, -FOOTBOT_BATT_GROUND_OFFSET - FOOTBOT_BATT_H / 2.0f, 0.0f),
         btVector3(0.0f, 1.0f, 0.0f), 
         btVector3(1.0f, 0.0f, 0.0f));
         
      m_vecLocalConstraints.push_back(m_pcLeftWheelToBodyConstraint);
      m_vecLocalConstraints.push_back(m_pcRightWheelToBodyConstraint);
         
      /** Create the pivots **/
      m_pcFrontPivotMotionState = new btDefaultMotionState(cEntityTransform * m_cFrontPivotTransform);
      m_pcRearPivotMotionState = new btDefaultMotionState(cEntityTransform * m_cRearPivotTransform);
      m_cPivotCollisionShape.calculateLocalInertia(FOOTBOT_PIVOT_MASS, cInteria);
      m_pcFrontPivotRigidBody = new btRigidBody(btRigidBody::btRigidBodyConstructionInfo(
         FOOTBOT_PIVOT_MASS, m_pcFrontPivotMotionState, &m_cPivotCollisionShape, cInteria));
      m_pcRearPivotRigidBody = new btRigidBody(btRigidBody::btRigidBodyConstructionInfo(
         FOOTBOT_PIVOT_MASS, m_pcRearPivotMotionState, &m_cPivotCollisionShape, cInteria));
         
      m_vecLocalRigidBodies.push_back(m_pcFrontPivotRigidBody);
      m_vecLocalRigidBodies.push_back(m_pcRearPivotRigidBody);
     
      /** create the pivots to base constraints **/
      m_pcFrontPivotToBodyConstraint = new btPoint2PointConstraint(
         *m_pcFrontPivotRigidBody,
         *m_pcBodyRigidBody,
         btVector3(0.0f, 0.0f, 0.0f),
         btVector3(0.0f, -FOOTBOT_BATT_GROUND_OFFSET - FOOTBOT_BATT_H / 2.0f, FOOTBOT_PIVOT_Z_OFFSET));          
      m_pcRearPivotToBodyConstraint = new btPoint2PointConstraint(
         *m_pcRearPivotRigidBody, 
         *m_pcBodyRigidBody,
         btVector3(0.0f, 0.0f, 0.0f),
         btVector3(0.0f, -FOOTBOT_BATT_GROUND_OFFSET - FOOTBOT_BATT_H / 2.0f, -FOOTBOT_PIVOT_Z_OFFSET));
         
      m_vecLocalConstraints.push_back(m_pcFrontPivotToBodyConstraint);
      m_vecLocalConstraints.push_back(m_pcRearPivotToBodyConstraint);
    
   }

   /****************************************/
   /****************************************/

   CDynamics3DFootBotEntity::~CDynamics3DFootBotEntity() {
      delete m_pcLeftWheelToBodyConstraint;
      delete m_pcRightWheelToBodyConstraint;
      delete m_pcFrontPivotToBodyConstraint;
      delete m_pcRearPivotToBodyConstraint;
      delete m_pcLeftWheelMotionState;
      delete m_pcRightWheelMotionState;
      delete m_pcFrontPivotMotionState;
      delete m_pcRearPivotMotionState;
      delete m_pcBodyMotionState;
      delete m_pcLeftWheelRigidBody;
      delete m_pcRightWheelRigidBody;
      delete m_pcFrontPivotRigidBody;
      delete m_pcRearPivotRigidBody;
      delete m_pcBodyRigidBody;
   }

   /****************************************/
   /****************************************/

   bool CDynamics3DFootBotEntity::CheckIntersectionWithRay(Real& f_t_on_ray,
                                                           const CRay3& c_ray) const {
      return false;
   }

   /****************************************/
   /****************************************/

   bool CDynamics3DFootBotEntity::MoveTo(const CVector3& c_position,
                                         const CQuaternion& c_orientation,
                                         bool b_check_only) {
      return false;
   }

   /****************************************/
   /****************************************/

   void CDynamics3DFootBotEntity::Reset() {
     
      btTransform cResetTransform(
         ARGoSToBullet(GetEmbodiedEntity().GetInitOrientation()),
         ARGoSToBullet(GetEmbodiedEntity().GetInitPosition())
      );
      
      m_pcLeftWheelRigidBody->getMotionState()->setWorldTransform(cResetTransform * m_cLeftWheelTransform);
      m_pcRightWheelRigidBody->getMotionState()->setWorldTransform(cResetTransform * m_cRightWheelTransform);
      m_pcFrontPivotRigidBody->getMotionState()->setWorldTransform(cResetTransform * m_cFrontPivotTransform);
      m_pcRearPivotRigidBody->getMotionState()->setWorldTransform(cResetTransform * m_cRearPivotTransform);
      m_pcBodyRigidBody->getMotionState()->setWorldTransform(cResetTransform * m_cBodyTransform);
      
      // clear velocities and forces on bodies
      for(std::vector<btRigidBody*>::iterator itBody = m_vecLocalRigidBodies.begin(); 
          itBody !=  m_vecLocalRigidBodies.end();
          itBody++) {   
         (*itBody)->setAngularVelocity(btVector3(0.0f, 0.0f, 0.0f));
         (*itBody)->setLinearVelocity(btVector3(0.0f, 0.0f, 0.0f));
         (*itBody)->clearForces();
      }
   }

   /****************************************/
   /****************************************/

   void CDynamics3DFootBotEntity::UpdateEntityStatus() {
      /* Update footbot position and orientation */
      btTransform cEntityTransform;
      
      m_pcBodyRigidBody->getMotionState()->getWorldTransform(cEntityTransform);
      
      cEntityTransform = m_cBodyTransform.inverse() * cEntityTransform;
      
      GetEmbodiedEntity().SetPosition(BulletToARGoS(cEntityTransform.getOrigin()));
      GetEmbodiedEntity().SetOrientation(BulletToARGoS(cEntityTransform.getRotation()));
      
      /* Update components */
      m_cFootBotEntity.UpdateComponents();
   }

   /****************************************/
   /****************************************/

   void CDynamics3DFootBotEntity::UpdateFromEntityStatus() {
      
      /* Get wheel speeds from entity */
      m_cWheeledEntity.GetSpeed(m_pfCurrentWheelVelocityFromSensor);

      /* Do we want to move? */
      if((m_pfCurrentWheelVelocityFromSensor[FOOTBOT_LEFT_WHEEL] != 0.0f) ||
         (m_pfCurrentWheelVelocityFromSensor[FOOTBOT_RIGHT_WHEEL] != 0.0f)) {
         
         Real fLeftWheelVelocity, fRightWheelVelocity;
         
         fLeftWheelVelocity  = m_pfCurrentWheelVelocityFromSensor[FOOTBOT_LEFT_WHEEL] / FOOTBOT_WHEEL_RADIUS;
         fRightWheelVelocity = m_pfCurrentWheelVelocityFromSensor[FOOTBOT_RIGHT_WHEEL] / FOOTBOT_WHEEL_RADIUS;
         
         // activate the bodies!
         for(std::vector<btRigidBody*>::iterator itBody = m_vecLocalRigidBodies.begin(); 
             itBody !=  m_vecLocalRigidBodies.end();
             itBody++) {   
            (*itBody)->activate();
         }
         
         /* because of the how the wheels are attached to the epuck the velocity on the right wheel speed is negated */
         m_pcLeftWheelToBodyConstraint->enableAngularMotor(true, fLeftWheelVelocity, FOOTBOT_WHEEL_MOTOR_IMPULSE);
         m_pcLeftWheelToBodyConstraint->enableAngularMotor(true, -fRightWheelVelocity, FOOTBOT_WHEEL_MOTOR_IMPULSE);

      }
      else {
         /* No, we don't want to move - zero all speeds */
         m_pcLeftWheelToBodyConstraint->enableAngularMotor(false, 0.0f, 0.0f);
         m_pcLeftWheelToBodyConstraint->enableAngularMotor(false, 0.0f, 0.0f);
      }
   }

   /****************************************/
   /****************************************/

   void CDynamics3DFootBotEntity::CalculateBoundingBox() {
      /** @todo Calculate foot-bot bounding box */
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_DYNAMICS3D_OPERATIONS_ON_ENTITY(CFootBotEntity, CDynamics3DFootBotEntity);

}

