/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_plugin.h>
 *
 * @author Haitham Elfaham - <haithamelfaham@gmail.com>
 */

#ifndef DYNAMICS3D_MAGNETISM_PLUGIN_H
#define DYNAMICS3D_MAGNETISM_PLUGIN_H

namespace argos {
   
}

#include <vector>
#include <string>
#include <iostream>
#include <sstream>
#include <math.h>
#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_plugin.h>
#include <argos3/core/utility/datatypes/datatypes.h>
#include <argos3/core/utility/datatypes/datatypes.h>

namespace argos {
   
   /****************************************/
   /****************************************/
   
   class CDynamics3DMagnetismPlugin : public CDynamics3DPlugin {
      
      
   public:
      
      CDynamics3DMagnetismPlugin() :
         CDynamics3DPlugin(), m_bDataStructureInitRequired(true) {}
      
      ~CDynamics3DMagnetismPlugin() {}
      
      virtual void Init(TConfigurationNode& t_tree);
      
      virtual void InitDataStructures();
      
      virtual void Reset() {}
      virtual void Destroy() {}

      virtual void RegisterModel(CDynamics3DModel& c_model) {}
      virtual void UnregisterModel(CDynamics3DModel& c_model) {}
      
      virtual void Update();
      
   private:
      
      
      struct SMagneticBody {
         
         
         CDynamics3DBody* Body;
         btVector3 Field;       
      };
      
      bool m_bDataStructureInitRequired;
      //      Real** m_ppfMinimumInterbodyDistance;
      
      Real m_fForceConstant;
      std::vector<SMagneticBody> m_vecMagneticBodies;
      
   };
   
   /****************************************/
   /****************************************/
   
}

#endif
