/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_plugin.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "dynamics3d_antigravity_plugin.h"

#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_model.h>
#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_body.h>

namespace argos {

   /****************************************/
   /****************************************/

   void CDynamics3DAntigravityPlugin::Init(TConfigurationNode& t_tree) {
      CVector3 cAntigravity;
      GetNodeAttribute(t_tree, "force", cAntigravity);
      m_cAntigravity = ARGoSToBullet(cAntigravity);
   }
   
   /****************************************/
   /****************************************/

   void CDynamics3DAntigravityPlugin::Update() {
      for(CDynamics3DModel::TVector::iterator itModel = m_pcEngine->GetModels().begin();
          itModel != m_pcEngine->GetModels().end();
          ++itModel) {
         for(CDynamics3DBody::TVector::iterator itBody = (*itModel)->GetBodies().begin();
             itBody != (*itModel)->GetBodies().end();
             ++itBody) {
            (*itBody)->ApplyForce(m_cAntigravity);
         }
      }
   }

   /****************************************/
   /****************************************/

   REGISTER_DYN3D_PHYSICS_PLUGIN(CDynamics3DAntigravityPlugin,
                                 "antigravity",
                                 "Michael Allwright [allsey87@gmail.com]",
                                 "1.0",
                                 "A plugin developed for testing the dynamics3d plugin mechanism",
                                 "This plugin applies a specified force to all bodies in the"
                                 "physics engine that it is applied to. The plugin has been "
                                 "developed for testing the dynamics3d plugin mechanism",
                                 "Under development");
}
