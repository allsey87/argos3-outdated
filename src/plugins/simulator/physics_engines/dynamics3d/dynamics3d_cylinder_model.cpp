/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_cylinder_model.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "dynamics3d_cylinder_model.h"

#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_engine.h>

namespace argos {

   /****************************************/
   /****************************************/

   CDynamics3DCylinderModel::CDynamics3DCylinderModel(CDynamics3DEngine& c_engine,
                                  CCylinderEntity& c_cylinder) :
      CDynamics3DModel(c_engine, c_cylinder.GetEmbodiedEntity()),
      m_cCylinderEntity(c_cylinder) {
      
      /* When defining size of objects we must manually swap the Z and Y components */
      m_pcCylinderCollisionShape = new btCylinderShape(btVector3(c_cylinder.GetRadius(),
                                                                 c_cylinder.GetHeight() * 0.5f,
                                                                 c_cylinder.GetRadius()));
      
      btTransform cModelTransform(ARGoSToBullet(GetEmbodiedEntity().GetInitOrientation()),
                                  ARGoSToBullet(GetEmbodiedEntity().GetInitPosition()));
      
      m_pcCylinderMotionState = new btDefaultMotionState(cModelTransform,
                                                         btTransform(btQuaternion(0.0f, 0.0f, 0.0f, 1.0f),
                                                                     btVector3(0.0f, -c_cylinder.GetHeight() * 0.5f, 0.0f)));
          
      if(c_cylinder.GetEmbodiedEntity().IsMovable()) {
         btVector3 cInertia;
         m_pcCylinderCollisionShape->calculateLocalInertia(c_cylinder.GetMass(), cInertia);
         m_pcCylinderRigidBody = new btRigidBody(btRigidBody::btRigidBodyConstructionInfo(
            c_cylinder.GetMass(), m_pcCylinderMotionState, m_pcCylinderCollisionShape, cInertia));
      }
      else {
         m_pcCylinderRigidBody = new btRigidBody(btRigidBody::btRigidBodyConstructionInfo(
            0.0f, m_pcCylinderMotionState, m_pcCylinderCollisionShape, btVector3(0.0f,0.0f,0.0f)));
      }
      m_mapLocalBodyConfigurations["cylinder"] = SBodyConfiguration(m_pcCylinderCollisionShape,
                                                                    m_pcCylinderMotionState,
                                                                    m_pcCylinderRigidBody);
   }
   
   /****************************************/
   /****************************************/
   
   CDynamics3DCylinderModel::~CDynamics3DCylinderModel() {
      delete m_pcCylinderRigidBody;
      delete m_pcCylinderMotionState;
      delete m_pcCylinderCollisionShape;
    }
   
   /****************************************/
   /****************************************/

   void CDynamics3DCylinderModel::UpdateEntityStatus() {
      if(m_cCylinderEntity.GetEmbodiedEntity().IsMovable()) {
         const btTransform& cUpdateTransform = GetModelCoordinates();
         
         GetEmbodiedEntity().SetPosition(BulletToARGoS(cUpdateTransform.getOrigin()));
         GetEmbodiedEntity().SetOrientation(BulletToARGoS(cUpdateTransform.getRotation()));

         m_cCylinderEntity.UpdateComponents();
      }
   }

   /****************************************/
   /****************************************/

   btTransform CDynamics3DCylinderModel::GetModelCoordinates() const {
      return m_pcCylinderMotionState->m_graphicsWorldTrans;
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_DYNAMICS3D_OPERATIONS_ON_ENTITY(CCylinderEntity, CDynamics3DCylinderModel);

}
