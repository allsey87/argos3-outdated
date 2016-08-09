/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics3d/plugins/gravity/dynamics3d_gravity_plugin.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "dynamics3d_gravity_plugin.h"
#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_model.h>
#include <algorithm>
#include <vector>

namespace argos {
   
   /****************************************/
   /****************************************/
   
   void CDynamics3DGravityPlugin::Init(TConfigurationNode& t_tree) {
      GetNodeAttributeOrDefault(t_tree, "g", m_fGravitationalAcceleration, m_fGravitationalAcceleration);
   } 

   /****************************************/
   /****************************************/

   void CDynamics3DGravityPlugin::RegisterModel(CDynamics3DModel& c_model) {
      m_tBodies.insert(m_tBodies.end(), c_model.GetBodies().begin(), c_model.GetBodies().end());
   }

   /****************************************/
   /****************************************/

   struct SRemoveOp : std::unary_function<CDynamics3DBody*, bool> {
      SRemoveOp(CDynamics3DModel& c_model) :
         Model(c_model) {}
      bool operator()(const CDynamics3DBody* pc_body) {
         return (pc_body->GetParentModel() == Model);
      }
      const CDynamics3DModel& Model;
   };

   /****************************************/
   /****************************************/

   void CDynamics3DGravityPlugin::UnregisterModel(CDynamics3DModel& c_model) {
      m_tBodies.erase(std::remove_if(m_tBodies.begin(), m_tBodies.end(), SRemoveOp(c_model)), m_tBodies.end());
   }
   
   /****************************************/
   /****************************************/

   void CDynamics3DGravityPlugin::Update() {
      for(CDynamics3DBody::TVectorIterator t_body_iter = m_tBodies.begin();
          t_body_iter != m_tBodies.end();
          t_body_iter++) {
         (*t_body_iter)->ApplyForce(btVector3(0.0, -m_fGravitationalAcceleration * (*t_body_iter)->GetMass(), 0.0));
      }
   }
   
   /****************************************/
   /****************************************/
   
   REGISTER_DYN3D_PHYSICS_PLUGIN(CDynamics3DGravityPlugin,
                                 "gravity",
                                 "Michael Allwright [allsey87@gmail.com]",
                                 "1.0",
                                 "A plugin developed for testing the dynamics3d plugin mechanism",
                                 "This plugin applies a specified force to all bodies in the"
                                 "physics engine that it is applied to. The plugin has been "
                                 "developed for testing the dynamics3d plugin mechanism",
                                 "Under development");
}
