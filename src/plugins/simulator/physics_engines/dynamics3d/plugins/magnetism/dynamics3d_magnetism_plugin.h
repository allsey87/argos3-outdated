/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_plugin.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef DYNAMICS3D_MAGNETISM_PLUGIN_H
#define DYNAMICS3D_MAGNETISM_PLUGIN_H

namespace argos {

}

#include <vector>
#include <string>

#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_plugin.h>
#include <argos3/core/utility/datatypes/datatypes.h>
#include <argos3/core/utility/datatypes/datatypes.h>

namespace argos {

   /****************************************/
   /****************************************/

   class CDynamics3DMagnetismPlugin : public CDynamics3DPlugin {


   public:

      CDynamics3DMagnetismPlugin() :
         CDynamics3DPlugin(),
         m_bDataStructureInitRequired(true),
         m_ppfMinimumInterbodyDistance(NULL) {}

      ~CDynamics3DMagnetismPlugin() {}

      virtual void Init(TConfigurationNode& t_tree);

virtual void InitDataStructures(CDynamics3DEngine& c_engine);

      virtual void Reset() {}
      virtual void Destroy() {}

      virtual void Update(CDynamics3DEngine& c_engine);

   private:

      struct SMagneticCell {
         btVector3 RelativePosition;
      };

      struct SMagneticBody {
         SMagneticBody(CDynamics3DBody* pc_body) :
            Body(pc_body) {}
         CDynamics3DBody* Body;
         std::vector<SMagneticCell> Cells;
      };

      bool m_bDataStructureInitRequired;

      std::vector<SMagneticBody> m_vecMagneticBodies;

      Real** m_ppfMinimumInterbodyDistance;
   };

   /****************************************/
   /****************************************/
   
}

#endif
