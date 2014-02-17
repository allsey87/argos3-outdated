/**
 * @file <argos3/plugins/robot/prototype/control_interface/ci_cameras_sensor/ci_cameras_sensor.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "ci_cameras_sensor.h"

#ifdef ARGOS_WITH_LUA
#include <argos3/core/wrappers/lua/lua_utility.h>
#endif

namespace argos {

   /****************************************/
   /****************************************/

   bool CCI_CamerasSensor::HasAlgorithm(const std::string& str_camera_name, const std::string& str_algorithm_type) const {
      std::map<std::string, CCI_CamerasSensorAlgorithm::TMap, std::less<std::string> >::const_iterator itCamera =
         m_mapAlgorithms.find(str_camera_name);
      if(itCamera != m_mapAlgorithms.end()) {
         CCI_CamerasSensorAlgorithm::TMap::const_iterator itAlgorithm = itCamera->second.find(str_algorithm_type);
         if(itAlgorithm != itCamera->second.end()) {
            return true;
         }
      }
      return false;
   }
   
   /****************************************/
   /****************************************/

#ifdef ARGOS_WITH_LUA
   int LuaEnableCamera(lua_State* pt_lua_state) {
      /* Perform action */
      CLuaUtility::GetDeviceInstance<CCI_CamerasSensor>(pt_lua_state, "cameras")->Enable();
         return 0;
   }
#endif

   /****************************************/
   /****************************************/

#ifdef ARGOS_WITH_LUA
   int LuaDisableCamera(lua_State* pt_lua_state) {
      /* Perform action */
      CLuaUtility::GetDeviceInstance<CCI_CamerasSensor>(pt_lua_state, "cameras")->Disable();
         return 0;
   }
#endif

   /****************************************/
   /****************************************/

#ifdef ARGOS_WITH_LUA
   void CCI_CamerasSensor::CreateLuaState(lua_State* pt_lua_state) {
      CLuaUtility::OpenRobotStateTable(pt_lua_state, "cameras");
      CLuaUtility::AddToTable(pt_lua_state, "_instance", this);
      //CLuaUtility::AddToTable(pt_lua_state, "enable", &LuaEnableCamera);
      //CLuaUtility::AddToTable(pt_lua_state, "disable", &LuaDisableCamera);
      for(std::map<std::string, CCI_CamerasSensorAlgorithm::TMap, std::less<std::string> >::iterator
             itCamera = m_mapAlgorithms.begin();
          itCamera != m_mapAlgorithms.end();
          ++itCamera) {
         CLuaUtility::StartTable(pt_lua_state, itCamera->first);
         for(CCI_CamerasSensorAlgorithm::TMap::iterator itAlgorithm = itCamera->second.begin();
             itAlgorithm != itCamera->second.end();
             ++itAlgorithm) {
            CLuaUtility::StartTable(pt_lua_state, itAlgorithm->first);
            itAlgorithm->second->CreateLuaState(pt_lua_state);
            CLuaUtility::EndTable(pt_lua_state);
         }
         CLuaUtility::EndTable(pt_lua_state);
      }
      CLuaUtility::CloseRobotStateTable(pt_lua_state);
   }
#endif

   /****************************************/
   /****************************************/

#ifdef ARGOS_WITH_LUA
   void CCI_CamerasSensor::ReadingsToLuaState(lua_State* pt_lua_state) {
      lua_getfield(pt_lua_state, -1, "cameras");
      for(std::map<std::string, CCI_CamerasSensorAlgorithm::TMap, std::less<std::string> >::iterator
             itCamera = m_mapAlgorithms.begin();
          itCamera != m_mapAlgorithms.end();
          ++itCamera) {
         CLuaUtility::StartTable(pt_lua_state, itCamera->first);
         for(CCI_CamerasSensorAlgorithm::TMap::iterator itAlgorithm = itCamera->second.begin();
             itAlgorithm != itCamera->second.end();
             ++itAlgorithm) {
            CLuaUtility::StartTable(pt_lua_state, itAlgorithm->first);
            itAlgorithm->second->ReadingsToLuaState(pt_lua_state);
            CLuaUtility::EndTable(pt_lua_state);
         }
         CLuaUtility::EndTable(pt_lua_state);
      }
      lua_pop(pt_lua_state, 1);
   }
#endif

   /****************************************/
   /****************************************/

}
