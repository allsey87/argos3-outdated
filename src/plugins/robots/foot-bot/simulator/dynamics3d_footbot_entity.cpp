/**
 * @file <argos3/plugins/robots/foot-bot/simulator/dynamics3d_footbot_entity.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "dynamics3d_footbot_entity.h"
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_engine.h>

namespace argos {

   static const Real FOOTBOT_BASE_W                =  0.100f;
   static const Real FOOTBOT_BASE_H                =  0.100f;
   static const Real FOOTBOT_BASE_L                =  0.100f;
   static const Real FOOTBOT_BASE_MASS             =  0.100f;

   static const Real FOOTBOT_WHEEL_RADIUS          =  0.025f;
   static const Real FOOTBOT_WHEEL_THICKNESS       =  0.010f;
   static const Real FOOTBOT_WHEEL_Y_OFFSET        = -0.050f;
   static const Real FOOTBOT_WHEEL_X_OFFSET        =  0.050f;
   static const Real FOOTBOT_WHEEL_MASS            =  0.001f;
   static const Real FOOTBOT_WHEEL_MOTOR_IMPULSE   =  1.500f;

   static const Real FOOTBOT_PIVOT_RADIUS          =  0.025f;
   static const Real FOOTBOT_PIVOT_Y_OFFSET        = -0.050f;
   static const Real FOOTBOT_PIVOT_Z_OFFSET        =  0.050f;
   static const Real FOOTBOT_PIVOT_MASS            =  0.001f;

   static const Real FOOTBOT_BODY_RADIUS           =  0.050f;
   static const Real FOOTBOT_BODY_THICKNESS        =  0.050f;
   static const Real FOOTBOT_BODY_Y_OFFSET         =  (FOOTBOT_BASE_H + FOOTBOT_BODY_THICKNESS) / 2.0f;
   static const Real FOOTBOT_BODY_MASS             =  0.100f;
   
   /****************************************/
   /****************************************/
   
   btBoxShape CDynamics3DFootBotEntity::m_cBaseCollisionShape(
      btVector3(FOOTBOT_BASE_W, FOOTBOT_BASE_H, FOOTBOT_BASE_L) / 2.0f);
   btCylinderShape CDynamics3DFootBotEntity::m_cWheelCollisionShape(
      btVector3(FOOTBOT_WHEEL_RADIUS, FOOTBOT_WHEEL_THICKNESS * 0.5f, FOOTBOT_WHEEL_RADIUS));
   btCylinderShape CDynamics3DFootBotEntity::m_cBodyCollisionShape(
      btVector3(FOOTBOT_BODY_RADIUS, FOOTBOT_BODY_THICKNESS * 0.5f, FOOTBOT_BODY_RADIUS));
   btSphereShape CDynamics3DFootBotEntity::m_cPivotCollisionShape(FOOTBOT_PIVOT_RADIUS);
   
   /****************************************/
   /****************************************/

   CDynamics3DFootBotEntity::CDynamics3DFootBotEntity(CDynamics3DEngine& c_engine,
                                                      CFootBotEntity& c_entity) :
      CDynamics3DEntity(c_engine, c_entity.GetEmbodiedEntity()),
      m_cFootBotEntity(c_entity),
      m_cWheeledEntity(m_cFootBotEntity.GetWheeledEntity()) {
      
      btVector3 cInteria;
      
      /** Create the base **/
      m_pcBaseMotionState = new btDefaultMotionState(btTransform(
         ARGoSToBullet(GetEmbodiedEntity().GetOrientation()),
         ARGoSToBullet(GetEmbodiedEntity().GetPosition())
      ));
      
      m_cBaseCollisionShape.calculateLocalInertia(FOOTBOT_BASE_MASS, cInteria);
      m_pcBaseRigidBody = new btRigidBody(btRigidBody::btRigidBodyConstructionInfo(
         FOOTBOT_BASE_MASS, m_pcBaseMotionState, &m_cBaseCollisionShape, cInteria));
      
      m_vecLocalRigidBodies.push_back(m_pcBaseRigidBody);
      
      /** create the wheels **/
      m_pcLeftWheelMotionState = new btDefaultMotionState;
      m_pcRightWheelMotionState = new btDefaultMotionState;
      m_cWheelCollisionShape.calculateLocalInertia(FOOTBOT_WHEEL_MASS, cInteria);
      m_pcLeftWheelRigidBody = new btRigidBody(btRigidBody::btRigidBodyConstructionInfo(
         FOOTBOT_WHEEL_MASS, m_pcLeftWheelMotionState, &m_cWheelCollisionShape, cInteria));
      m_pcRightWheelRigidBody = new btRigidBody(btRigidBody::btRigidBodyConstructionInfo(
         FOOTBOT_WHEEL_MASS, m_pcRightWheelMotionState, &m_cWheelCollisionShape, cInteria));
      
      m_vecLocalRigidBodies.push_back(m_pcLeftWheelRigidBody);
      m_vecLocalRigidBodies.push_back(m_pcRightWheelRigidBody);
      
      /** create the wheels to base constraints **/
      m_pcLeftWheelToBaseConstraint = new btHingeConstraint(
         *m_pcLeftWheelRigidBody,
         *m_pcBaseRigidBody,
         btVector3(0.0f, 0.0f, 0.0f),
         btVector3(-FOOTBOT_WHEEL_X_OFFSET, FOOTBOT_WHEEL_Y_OFFSET, 0.0f),
         btVector3(0.0f, 1.0f, 0.0f), 
         btVector3(-1.0f, 0.0f, 0.0f));
      m_pcRightWheelToBaseConstraint = new btHingeConstraint(
         *m_pcRightWheelRigidBody, 
         *m_pcBaseRigidBody,
         btVector3(0.0f, 0.0f, 0.0f),
         btVector3(FOOTBOT_WHEEL_X_OFFSET, FOOTBOT_WHEEL_Y_OFFSET, 0.0f),
         btVector3(0.0f, 1.0f, 0.0f), 
         btVector3(1.0f, 0.0f, 0.0f));
         
      m_vecLocalConstraints.push_back(m_pcLeftWheelToBaseConstraint);
      m_vecLocalConstraints.push_back(m_pcRightWheelToBaseConstraint);
         
      /** Create the pivots **/
      m_pcFrontPivotMotionState = new btDefaultMotionState;
      m_pcRearPivotMotionState = new btDefaultMotionState;
      m_cPivotCollisionShape.calculateLocalInertia(FOOTBOT_PIVOT_MASS, cInteria);
      m_pcFrontPivotRigidBody = new btRigidBody(btRigidBody::btRigidBodyConstructionInfo(
         FOOTBOT_PIVOT_MASS, m_pcFrontPivotMotionState, &m_cPivotCollisionShape, cInteria));
      m_pcRearPivotRigidBody = new btRigidBody(btRigidBody::btRigidBodyConstructionInfo(
         FOOTBOT_PIVOT_MASS, m_pcRearPivotMotionState, &m_cPivotCollisionShape, cInteria));
         
      m_vecLocalRigidBodies.push_back(m_pcFrontPivotRigidBody);
      m_vecLocalRigidBodies.push_back(m_pcRearPivotRigidBody);
     
      /** create the pivots to base constraints **/
      m_pcFrontPivotToBaseConstraint = new btPoint2PointConstraint(
         *m_pcFrontPivotRigidBody,
         *m_pcBaseRigidBody,
         btVector3(0.0f, 0.0f, 0.0f),
         btVector3(0.0f, FOOTBOT_PIVOT_Y_OFFSET, FOOTBOT_PIVOT_Z_OFFSET));          
      m_pcRearPivotToBaseConstraint = new btPoint2PointConstraint(
         *m_pcRearPivotRigidBody, 
         *m_pcBaseRigidBody,
         btVector3(0.0f, 0.0f, 0.0f),
         btVector3(0.0f, FOOTBOT_PIVOT_Y_OFFSET, -FOOTBOT_PIVOT_Z_OFFSET));
         
      m_vecLocalConstraints.push_back(m_pcFrontPivotToBaseConstraint);
      m_vecLocalConstraints.push_back(m_pcRearPivotToBaseConstraint);
         
      /** create the body **/
      m_pcBodyMotionState = new btDefaultMotionState;
      m_cBodyCollisionShape.calculateLocalInertia(FOOTBOT_BODY_MASS, cInteria);
      m_pcBodyRigidBody = new btRigidBody(btRigidBody::btRigidBodyConstructionInfo(
         FOOTBOT_BODY_MASS, m_pcBodyMotionState, &m_cBodyCollisionShape, cInteria));
         
      m_vecLocalRigidBodies.push_back(m_pcBodyRigidBody);
         
      /** create the body to base constraint **/
      m_pcBaseToBodyConstraint = new btHingeConstraint(
         *m_pcBaseRigidBody,
         *m_pcBodyRigidBody,
         btVector3(0.0f, FOOTBOT_BODY_Y_OFFSET / 2.0f, 0.0f),
         btVector3(0.0f, -FOOTBOT_BODY_Y_OFFSET / 2.0f, 0.0f),
         btVector3(0.0f, 1.0f, 0.0f), 
         btVector3(0.0f, 1.0f, 0.0f));
      
      m_pcBaseToBodyConstraint->setLimit(-0.0f, 0.0f);
      
      m_vecLocalConstraints.push_back(m_pcBaseToBodyConstraint);
      
      //TODO combine base and body bodies and constraints with a single compound RB
   }

   /****************************************/
   /****************************************/

   CDynamics3DFootBotEntity::~CDynamics3DFootBotEntity() {
      delete m_pcLeftWheelToBaseConstraint;
      delete m_pcRightWheelToBaseConstraint;
      delete m_pcBaseToBodyConstraint;
      delete m_pcFrontPivotToBaseConstraint;
      delete m_pcRearPivotToBaseConstraint;
      delete m_pcBaseMotionState;
      delete m_pcLeftWheelMotionState;
      delete m_pcRightWheelMotionState;
      delete m_pcFrontPivotMotionState;
      delete m_pcRearPivotMotionState;
      delete m_pcBodyMotionState;
      delete m_pcBaseRigidBody;
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
      for(std::vector<btRigidBody*>::iterator itBody = m_vecLocalRigidBodies.begin(); 
          itBody !=  m_vecLocalRigidBodies.end();
          itBody++) {   
         (*itBody)->setAngularVelocity(btVector3(0.0f, 0.0f, 0.0f));
         (*itBody)->setLinearVelocity(btVector3(0.0f, 0.0f, 0.0f));
         (*itBody)->clearForces();
      
         (*itBody)->getMotionState()->setWorldTransform(btTransform::getIdentity());
      }
      
      m_pcBaseRigidBody->setWorldTransform(btTransform(
            ARGoSToBullet(GetEmbodiedEntity().GetInitOrientation()),
            ARGoSToBullet(GetEmbodiedEntity().GetInitPosition())
      ));
      
      UpdateEntityStatus();
   }

   /****************************************/
   /****************************************/

   void CDynamics3DFootBotEntity::UpdateEntityStatus() {
      /* Update footbot position and orientation */
      btTransform cEntityTransform;
      m_pcBaseMotionState->getWorldTransform(cEntityTransform);
      GetEmbodiedEntity().SetPosition(BulletToARGoS(cEntityTransform.getOrigin()));
      GetEmbodiedEntity().SetOrientation(BulletToARGoS(cEntityTransform.getRotation()));
      
      /* Update components */
      m_cFootBotEntity.UpdateComponents();
   }

   /****************************************/
   /****************************************/

   void CDynamics3DFootBotEntity::UpdateFromEntityStatus() {
   
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

