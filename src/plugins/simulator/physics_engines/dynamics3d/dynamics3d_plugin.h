/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_plugin.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef DYNAMICS3D_PLUGIN_H
#define DYNAMICS3D_PLUGIN_H

namespace argos {

}

#include <argos3/core/utility/datatypes/datatypes.h>
#include <argos3/core/utility/plugins/factory.h>
#include <argos3/core/simulator/simulator.h>

#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_engine.h>

namespace argos {

   /****************************************/
   /****************************************/

   class CDynamics3DPlugin {

   public:

      typedef std::vector<CDynamics3DPlugin*> TVector;

   public:

      CDynamics3DPlugin() {}

      virtual ~CDynamics3DPlugin() {}
      
      virtual void Init(TConfigurationNode& t_tree) {}
      virtual void Reset() {}
      virtual void Destroy() {}

      virtual void SetEngine(CDynamics3DEngine& c_engine) {
         m_pcEngine = &c_engine;
      }

      virtual void RegisterModel(CDynamics3DModel& c_model) = 0;
      virtual void UnregisterModel(CDynamics3DModel& c_model) = 0;

      virtual void Update() = 0;

      const std::string& GetId() const {
         return m_strId;
      }

   protected:
      CDynamics3DEngine* m_pcEngine;
      
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
