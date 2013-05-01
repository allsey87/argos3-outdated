/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_body.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef DYNAMICS3D_BODY_H
#define DYNAMICS3D_BODY_H

namespace argos {

}

#include <vector>
#include <string>

#include <argos3/plugins/simulator/physics_engines/dynamics3d/bullet/btBulletDynamicsCommon.h>
#include <argos3/core/utility/datatypes/datatypes.h>

namespace argos {
   
   /****************************************/
   /****************************************/

   class CDynamics3DBody {

   public:

      class TVector : public std::vector<CDynamics3DBody*> {
      public:
         iterator Find(const std::string& str_id);
         const_iterator Find(const std::string& str_id) const;
         CDynamics3DBody* operator[](const std::string& str_id);
         const CDynamics3DBody* operator[](const std::string& str_id) const;
         CDynamics3DBody* operator[](UInt32 un_idx);
         const CDynamics3DBody* operator[](UInt32 un_idx) const;
      };

   public:

      CDynamics3DBody(const std::string& str_id,
                      btCollisionShape* pc_collision_shape = NULL,
                      const btTransform& c_positional_offset = btTransform::getIdentity(),
                      const btTransform& c_geometric_offset = btTransform::getIdentity(),
                      Real f_mass = 0.0f);

      ~CDynamics3DBody();

      void Reset();

      const std::string& GetId() {
         return m_strId;
      }

      const btCollisionShape& GetCollisionShape() const;

      bool operator==(const btCollisionObject* pc_collision_object) const;

      const btTransform& GetRigidBodyTransform() const;    

      const btTransform& GetPositionalOffset() const;

      const btTransform& GetMotionStateTransform() const;
      
      void SetMotionStateTransform(const btTransform & cTransform);

      void SynchronizeMotionState();

      void ActivateRigidBody();

      void AddBodyToWorld(btDynamicsWorld * pc_dynamics_world);
      void RemoveBodyFromWorld(btDynamicsWorld * pc_dynamics_world);

   private:
      std::string m_strId;

      btCollisionShape* m_pcCollisionShape;
      btDefaultMotionState* m_pcMotionState;
      btRigidBody* m_pcRigidBody;

      const btTransform m_cGeometricOffset;
      const btTransform m_cPositionalOffset;
      
      btVector3 m_cInertia;
      Real m_fMass;


      friend class CDynamics3DJoint;
   };
}

#endif
