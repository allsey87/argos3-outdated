/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_plugin.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef DYNAMICS3D_PLUGIN_H
#define DYNAMICS3D_PLUGIN_H

namespace argos {

}

#include <vector>
#include <string>

#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_engine.h>
//#include <argos3/plugins/simulator/physics_engines/dynamics3d/bullet/btBulletDynamicsCommon.h>
#include <argos3/core/utility/datatypes/datatypes.h>
#include <argos3/core/utility/plugins/factory.h>

namespace argos {

   /****************************************/
   /****************************************/

   class CDynamics3DPlugin {

   public:

      typedef std::vector<CDynamics3DPlugin*> TVector;

   public:

      CDynamics3DPlugin() {}

      ~CDynamics3DPlugin() {}

      virtual void Init(TConfigurationNode& t_tree) {}
      virtual void Reset() {}
      virtual void Destroy() {}

      virtual void Update(CDynamics3DEngine& c_engine) = 0;

      const std::string& GetId() const {
         return m_strId;
      }

   protected:
      std::string m_strId;
   };

   /****************************************/
   /****************************************/

   bool operator==(const CDynamics3DPlugin* pc_dyn3d_plugin, const std::string& str_id);

   /****************************************/
   /****************************************/
   
}

#define REGISTER_DYN3D_PHYSICS_PLUGIN(CLASSNAME,          \
                                      LABEL,              \
                                      AUTHOR,             \
                                      VERSION,            \
                                      BRIEF_DESCRIPTION,  \
                                      LONG_DESCRIPTION,   \
                                      STATUS)             \
   REGISTER_SYMBOL(CDynamics3DPlugin,               \
                   CLASSNAME,                       \
                   LABEL,                           \
                   AUTHOR,                          \
                   VERSION,                         \
                   BRIEF_DESCRIPTION,               \
                   LONG_DESCRIPTION,                \
                   STATUS)

#endif
