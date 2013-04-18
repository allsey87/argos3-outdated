/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_model.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef DYNAMICS3D_MODEL_H
#define DYNAMICS3D_MODEL_H

namespace argos {
   class  CDynamics3DEngine;
}

#include <argos3/core/simulator/physics_engine/physics_model.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_engine.h>

#include <tr1/unordered_map>

namespace argos {

   /****************************************/
   /****************************************/
   
   inline CVector3 BulletToARGoS(const btVector3& c_bt_vector) {
      return CVector3(c_bt_vector.getX(), -c_bt_vector.getZ(), c_bt_vector.getY());
   }
   
   inline btVector3 ARGoSToBullet(const CVector3& c_a_vector) {
      return btVector3(c_a_vector.GetX(), c_a_vector.GetZ(), -c_a_vector.GetY());
   }
   
   inline CQuaternion BulletToARGoS(const btQuaternion& c_bt_quaternion) {
      return CQuaternion(c_bt_quaternion.getW(), c_bt_quaternion.getX(),
                         -c_bt_quaternion.getZ(), c_bt_quaternion.getY());
   }
   
   inline btQuaternion ARGoSToBullet(const CQuaternion& c_a_quaternion) {
      return btQuaternion(c_a_quaternion.GetX(), c_a_quaternion.GetZ(), 
                          -c_a_quaternion.GetY(), c_a_quaternion.GetW());
   }
   
   /****************************************/
   /****************************************/

   class CDynamics3DModel : public CPhysicsModel {

   public:
      
      typedef std::vector<CDynamics3DModel*> TVector;
      typedef std::map<std::string, CDynamics3DModel*> TMap;
      typedef std::tr1::unordered_multimap<std::string, CDynamics3DModel*> TMultiMap;

   public:

      struct SBodyConfiguration {

         SBodyConfiguration(const std::string& str_id = "",
                            btCollisionShape* pc_collision_shape = NULL,
                            btDefaultMotionState* pc_motion_state = NULL,
                            btRigidBody* pc_rigid_body = NULL,
                            btTransform c_offset_transform = btTransform::getIdentity(),
                            btVector3 c_inertia = btVector3(0.0f,0.0f,0.0f),
                            Real f_mass = 0.0f) :
            m_strId(str_id),
            m_pcCollisionShape(pc_collision_shape),
            m_pcMotionState(pc_motion_state),
            m_pcRigidBody(pc_rigid_body),
            m_cOffsetTransform(c_offset_transform),
            m_cInertia(c_inertia),
            m_fMass(f_mass) {}

         std::string m_strId;
         btCollisionShape* m_pcCollisionShape;
         btDefaultMotionState* m_pcMotionState;
         btRigidBody* m_pcRigidBody;
         btTransform m_cOffsetTransform;
         btVector3 m_cInertia;
         Real m_fMass;
      };

      struct SConstraint {
         SConstraint(const std::string& str_id = "",
                     btTypedConstraint* pc_constraint = NULL,
                     bool b_disable_collisions = false) : 
            m_strId(str_id),
            m_pcConstraint(pc_constraint),
            m_bDisableCollisions(b_disable_collisions) {}
         
         std::string m_strId;
         btTypedConstraint* m_pcConstraint;
         bool m_bDisableCollisions;
      };

   public:

      CDynamics3DModel(CDynamics3DEngine& c_engine,
                       CEmbodiedEntity& c_entity) :
         CPhysicsModel(c_engine, c_entity),
         m_cEngine(c_engine) {}
      virtual ~CDynamics3DModel() {}

      virtual bool MoveTo(const CVector3& c_position,
                          const CQuaternion& c_orientation,
                          bool b_check_only = false);

      virtual void Reset();
      
      virtual void UpdateEntityStatus() = 0;

      virtual void UpdateFromEntityStatus() = 0;
      
      inline const std::vector<SBodyConfiguration>& GetBodies() const {
         return m_vecLocalBodyConfigurations;
      }
      
      inline const std::vector<SConstraint>& GetConstraints() const {
         return m_vecLocalConstraints;
      }

      virtual void CalculateBoundingBox();

      virtual bool IsCollidingWithSomething() const;

      virtual bool CheckIntersectionWithRay(Real& f_t_on_ray,
                                            const CRay3& c_ray) const;

   protected:

      //virtual void Setup() = 0;

      virtual btTransform GetModelCoordinates() const = 0;

      virtual void SetModelCoordinates(const btTransform& c_coordinates);

   protected:

      CDynamics3DEngine&      m_cEngine;

      class : public std::vector<SBodyConfiguration> {
      public:
         const SBodyConfiguration& Find(const std::string& str_id) const {
            std::vector<SBodyConfiguration>::const_iterator it;
            
            for(it = this->begin(); it != this->end(); ++it) {
               if(it->m_strId == str_id) break;
            }
            if(it == this->end()) {
               THROW_ARGOSEXCEPTION("Could not find a body with ID \"" << str_id << "\".");
            }
            return *it;
         }
      } m_vecLocalBodyConfigurations;


      class : public std::vector<SConstraint> {
      public:
         const SConstraint& Find(const std::string& str_id) const {
            std::vector<SConstraint>::const_iterator it;
            
            for(it = this->begin(); it != this->end(); ++it) {
               if(it->m_strId == str_id) break;
            }
            if(it == this->end()) {
               THROW_ARGOSEXCEPTION("Could not find a constraint with ID \"" << str_id << "\".");
            }
            return *it;
         }
      } m_vecLocalConstraints;
   };
}

#endif
