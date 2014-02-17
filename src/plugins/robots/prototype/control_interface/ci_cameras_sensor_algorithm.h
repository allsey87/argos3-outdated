/**
 * @file <argos3/plugins/robot/prototype/control_interface/ci_cameras_sensor_algorithm.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef CI_CAMERAS_SENSOR_ALGORITHM_H
#define CI_CAMERAS_SENSOR_ALGORITHM_H

namespace argos {
	class CCI_CamerasSensorAlgorithm;
}

#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/configuration/base_configurable_resource.h>

#include <map>
#include <string>

#ifdef ARGOS_WITH_LUA
extern "C" {
#include <lua.h>
#include <lualib.h>
#include <lauxlib.h>
}
#endif

namespace argos {
   
   class CCI_CamerasSensorAlgorithm : public CBaseConfigurableResource {
      
   public:

      typedef std::map<std::string, CCI_CamerasSensorAlgorithm*, std::less<std::string> > TMap;

   public:
      
      virtual ~CCI_CamerasSensorAlgorithm() {}

      virtual void Init(TConfigurationNode& t_node) {}
      
      virtual void Reset() {}
      
      virtual void Destroy() {}

#ifdef ARGOS_WITH_LUA
      virtual void CreateLuaState(lua_State* pt_lua_state) = 0;
      
      virtual void ReadingsToLuaState(lua_State* pt_lua_state) = 0;
#endif
           
   };
   
}

#endif
