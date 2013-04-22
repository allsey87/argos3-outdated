/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_model.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef DYNAMICS3D_BODY_H
#define DYNAMICS3D_BODY_H

namespace argos {
   class  CDynamics3DEngine;
}

#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/quaternion.h>

namespace argos {
   
   /****************************************/
   /****************************************/

   class CDynamics3DBody {

   public:

      class TVector : public std::vector<CDynamics3DBody> {
      public:
         const CDynamics3DBody& Find(const std::string& str_id) const {
            std::vector<CDynamics3DBody>::const_iterator it;
            
            for(it = this->begin(); it != this->end(); ++it) {
               if(it->m_strId == str_id) break;
            }
            if(it == this->end()) {
               THROW_ARGOSEXCEPTION("Could not find a body with ID \"" << str_id << "\".");
            }
            return *it;
         }
      };

   public:

      CDynamics3DBody(const std::string& str_id = "",
                      btCollisionShape* pc_collision_shape = NULL,
                      btDefaultMotionState* pc_motion_state = NULL,
                      btRigidBody* pc_rigid_body = NULL,
                      btTransform c_positional_offset = btTransform::getIdentity(),
                      btTransform c_geometric_offset = btTransform::getIdentity(),
                      btVector3 c_inertia = btVector3(0.0f,0.0f,0.0f), 
                      //@todo remove inertia, calculate in constructor
                      Real f_mass = 0.0f) :
         m_strId(str_id),
         m_pcCollisionShape(pc_collision_shape),
         m_pcMotionState(pc_motion_state),
         m_pcRigidBody(pc_rigid_body),
         m_cGeometricOffset(c_geometric_offset),
         m_cPositionalOffset(c_positional_offset),
         m_cInertia(c_inertia),
         m_fMass(f_mass) {}

      //@todo calculate inertia here, if mass equals 0.0f then interia equals (0,0,0)
      // call reset to complete the construction?


      void Reset() {
         // recreate the motion state
         delete m_pcMotionState;
         m_pcMotionState = new btDefaultMotionState(m_cPositionalOffset, m_cGeometricOffset);

         // delete the body and recreate it
         delete m_pcRigidBody;
         m_pcRigidBody = new btRigidBody(btRigidBody::btRigidBodyConstructionInfo(m_fMass,
                                                                                  m_pcMotionState,
                                                                                  m_pcCollisionShape,
                                                                                  m_cInertia));
      }
    
   public: //@todo make this private add getters setters

      std::string m_strId;
      btCollisionShape* m_pcCollisionShape;
      btDefaultMotionState* m_pcMotionState;
      btRigidBody* m_pcRigidBody;

      btTransform m_cGeometricOffset;
      btTransform m_cPositionalOffset;
      btVector3 m_cInertia;

      Real m_fMass;

   };
}

#endif
