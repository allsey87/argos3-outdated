/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_plugin.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "dynamics3d_plugin.h"

namespace argos {
  
   /****************************************/
   /****************************************/
  
   bool operator==(const CDynamics3DPlugin* pc_dyn3d_plugin, const std::string& str_id) {
      return (pc_dyn3d_plugin->GetId()) == str_id;
   }

   /****************************************/
   /****************************************/

}
