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
#include <map>

#include <argos3/plugins/simulator/physics_engines/dynamics3d/bullet/btBulletDynamicsCommon.h>
#include <argos3/core/utility/datatypes/datatypes.h>

namespace argos {

   /****************************************/
   /****************************************/

   class CDynamics3DBody {

   public:

      typedef std::vector<CDynamics3DBody*> TVector;
      typedef std::map<std::string, std::string> TAttributesMap;

   public:

      CDynamics3DBody(const std::string& str_id,
                      btCollisionShape* pc_collision_shape = NULL,
                      const btTransform& c_positional_offset = btTransform::getIdentity(),
                      const btTransform& c_geometric_offset = btTransform::getIdentity(),
                      Real f_mass = 0.0f,
                      const TAttributesMap& map_attributes = TAttributesMap());

      ~CDynamics3DBody();

      void Reset();

      const std::string& GetId() const {
         return m_strId;
      }

      const std::string& GetAttribute(const std::string& str_key) const;

      bool HasAttribute(const std::string& str_key) const;

      const btCollisionShape& GetCollisionShape() const;

      bool operator==(const btCollisionObject* pc_collision_object) const;

      const btTransform& GetRigidBodyTransform() const;    

      const btTransform& GetPositionalOffset() const;

      const btTransform& GetGeometricOffset() const;

      const btTransform& GetMotionStateTransform() const;
      
      void SetMotionStateTransform(const btTransform& c_transform);

      void SynchronizeMotionState();

      void ActivateRigidBody();

      void ApplyForce(const btVector3& c_force, const btVector3& c_offset = btVector3(0.0f, 0.0f, 0.0f));

      const btVector3& GetTotalForce() const;

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

      std::map<std::string, std::string> m_mapAttributes;

      friend class CDynamics3DJoint;
   };

   /****************************************/
   /****************************************/

   bool operator==(const CDynamics3DBody* pc_dyn3d_body, const std::string& str_id);

   /****************************************/
   /****************************************/
   
}

#endif
