/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_plugin.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef DYNAMICS3D_ANTIGRAVITY_PLUGIN_H
#define DYNAMICS3D_ANTIGRAVITY_PLUGIN_H

namespace argos {

}

#include <vector>
#include <string>

#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_plugin.h>
//#include <argos3/plugins/simulator/physics_engines/dynamics3d/bullet/btBulletDynamicsCommon.h>
#include <argos3/core/utility/datatypes/datatypes.h>

namespace argos {

   /****************************************/
   /****************************************/

   class CDynamics3DAntigravityPlugin : public CDynamics3DPlugin {


   public:

      CDynamics3DAntigravityPlugin() :
         CDynamics3DPlugin() {}

      ~CDynamics3DAntigravityPlugin() {}

      virtual void Init(TConfigurationNode& t_tree);

      virtual void Reset() {}
      virtual void Destroy() {}

      virtual void RegisterModel(CDynamics3DModel& c_model) {}
      virtual void UnregisterModel(CDynamics3DModel& c_model) {}

      virtual void Update();

   private:

      btVector3 m_cAntigravity;

   };

   /****************************************/
   /****************************************/
   
}

#endif
