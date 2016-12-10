/**
 * @file <argos3/plugins/robot/prototype/control_interface/ci_cameras_sensor_algorithms/ci_cameras_sensor_tagdetector_algorithm.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "ci_cameras_sensor_tagdetector_algorithm.h"

#ifdef ARGOS_WITH_LUA
#include <argos3/core/wrappers/lua/lua_utility.h>
#endif


namespace argos {

   /****************************************/
   /****************************************/

#ifdef ARGOS_WITH_LUA
   void CCI_CamerasSensorTagDetectorAlgorithm::CreateLuaState(lua_State* pt_lua_state) {
      for(size_t i = 0; i < m_tReadings.size(); ++i) {
         CLuaUtility::StartTable(pt_lua_state, i+1);
         // @todo Implement CLuaUtility::AddToTable(pt_lua_state*, const char *, const std::string&);
         //CLuaUtility::AddToTable(pt_lua_state, "payload", m_tReadings[i].Payload);
         lua_pushstring(pt_lua_state, "payload");
         lua_pushstring(pt_lua_state, m_tReadings[i].Payload.c_str());
         lua_settable  (pt_lua_state, -3);
         //
         
         CLuaUtility::AddToTable(pt_lua_state, "center", m_tReadings[i].Center);
         CLuaUtility::EndTable(pt_lua_state);
      }
   }
#endif

   /****************************************/
   /****************************************/

#ifdef ARGOS_WITH_LUA
   void CCI_CamerasSensorTagDetectorAlgorithm::ReadingsToLuaState(lua_State* pt_lua_state) {
      // @todo lua_objlen(pt_lua_state, -1) is returning always 0 instead of last number of readings
      size_t unLastReadingsNum = lua_objlen(pt_lua_state, -1);
      for(size_t i = 0; i < m_tReadings.size(); ++i) {
         CLuaUtility::StartTable(pt_lua_state, i+1);         
         // @todo Implement CLuaUtility::AddToTable(pt_lua_state*, const char *, const std::string&);
         //CLuaUtility::AddToTable(pt_lua_state, "payload", m_tReadings[i].Payload);
         lua_pushstring(pt_lua_state, "payload");
         lua_pushstring(pt_lua_state, m_tReadings[i].Payload.c_str());
         lua_settable  (pt_lua_state, -3);
         //
         CLuaUtility::AddToTable(pt_lua_state, "center", m_tReadings[i].Center);
         CLuaUtility::EndTable(pt_lua_state);
      }
      if(m_tReadings.size() < unLastReadingsNum) {
         /* Yes, set to nil all the extra entries */
         for(size_t i = m_tReadings.size() + 1; i <= unLastReadingsNum; ++i) {
            lua_pushnumber(pt_lua_state,  i);
            lua_pushnil   (pt_lua_state    );
            lua_settable  (pt_lua_state, -3);
         }
      }
   } 
#endif

   /****************************************/
   /****************************************/
   
}
