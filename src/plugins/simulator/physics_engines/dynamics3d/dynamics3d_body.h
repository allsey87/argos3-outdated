/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_body.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef DYNAMICS3D_BODY_H
#define DYNAMICS3D_BODY_H

namespace argos {
   class CDynamics3DModel;
}

#include <vector>
#include <string>
#include <map>

#include <argos3/plugins/simulator/physics_engines/dynamics3d/bullet/btBulletDynamicsCommon.h>
#include <argos3/core/utility/datatypes/datatypes.h>


namespace argos {

   /****************************************/
   /****************************************/

   class CDynamics3DBody {
   public:
      typedef std::vector<CDynamics3DBody*> TVector;
      typedef std::vector<CDynamics3DBody*>::iterator TVectorIterator;
      typedef std::vector<CDynamics3DBody*>::const_iterator TVectorConstIterator;

   public:
      CDynamics3DBody(CDynamics3DModel* pc_parent_model,
                      const std::string& str_id,
                      btCollisionShape* pc_collision_shape = NULL,
                      const btTransform& c_positional_offset = btTransform::getIdentity(),
                      const btTransform& c_geometric_offset = btTransform::getIdentity(),
                      btScalar f_mass = 0.0f);

      ~CDynamics3DBody();

      void Reset();

      const std::string& GetId() const {
         return m_strId;
      }

      CDynamics3DModel& GetParentModel() {
         return *m_pcParentModel;
      }

      const CDynamics3DModel& GetParentModel() const {
         return *m_pcParentModel;
      }
   
      bool HasParentModel() const {
         return (m_pcParentModel != NULL);
      }

      const btCollisionShape& GetCollisionShape() const;

      const btTransform& GetRigidBodyTransform() const;    

      const btTransform& GetPositionalOffset() const;

      const btTransform& GetGeometricOffset() const;

      const btTransform& GetMotionStateTransform() const;

      btScalar GetMass() const; 
      
      void SetMotionStateTransform(const btTransform& c_transform);

      void SetDamping(btScalar f_linear_damping, btScalar f_angular_damping);

      void SynchronizeMotionState();

      void ActivateRigidBody();

      bool IsRigidBodyActive();

      void ApplyTorque(const btVector3& c_torque);

      void ApplyForce(const btVector3& c_force, const btVector3& c_offset = btVector3(0.0f, 0.0f, 0.0f));

      const btVector3& GetTotalForce() const;

      void AddBodyToWorld(btDynamicsWorld * pc_dynamics_world);
      void RemoveBodyFromWorld(btDynamicsWorld * pc_dynamics_world);

   public:
      CDynamics3DModel* m_pcParentModel;
      std::string m_strId;

      btCollisionShape* m_pcCollisionShape;
      btDefaultMotionState* m_pcMotionState;
      btRigidBody* m_pcRigidBody;

      const btTransform m_cGeometricOffset;
      const btTransform m_cPositionalOffset;
      
      btVector3 m_cInertia;
      btScalar m_fMass;

      friend class CDynamics3DJoint;
   };

   /****************************************/
   /****************************************/

   bool operator==(const CDynamics3DBody* pc_dyn3d_body, const std::string& str_id);

   /****************************************/
   /****************************************/
   
}

#endif
