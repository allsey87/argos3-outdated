/**
 * @file <argos3/plugins/robot/prototype/control_interface/ci_cameras_sensor_algorithms/ci_cameras_sensor_leddetector_algorithm.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "ci_cameras_sensor_leddetector_algorithm.h"

#ifdef ARGOS_WITH_LUA
#include <argos3/core/wrappers/lua/lua_utility.h>
#endif


namespace argos {

   /****************************************/
   /****************************************/

#ifdef ARGOS_WITH_LUA
   void CCI_CamerasSensorLEDDetectorAlgorithm::CreateLuaState(lua_State* pt_lua_state) {
      for(size_t i = 0; i < m_tReadings.size(); ++i) {
         CLuaUtility::StartTable(pt_lua_state, i+1);
         CLuaUtility::AddToTable(pt_lua_state, "color", m_tReadings[i].Color);
         CLuaUtility::AddToTable(pt_lua_state, "horizontal_idx", m_tReadings[i].HorizontalIndex);
         CLuaUtility::AddToTable(pt_lua_state, "vertical_idx", m_tReadings[i].VerticalIndex);
         CLuaUtility::EndTable(pt_lua_state);
      }
   }
#endif

   /****************************************/
   /****************************************/

#ifdef ARGOS_WITH_LUA
   void CCI_CamerasSensorLEDDetectorAlgorithm::ReadingsToLuaState(lua_State* pt_lua_state) {
      // @todo lua_objlen(pt_lua_state, -1) is returning always 0 instead of last number of readings
      size_t unLastReadingsNum = lua_objlen(pt_lua_state, -1);
      for(size_t i = 0; i < m_tReadings.size(); ++i) {
         CLuaUtility::StartTable(pt_lua_state, i+1);
         CLuaUtility::AddToTable(pt_lua_state, "color", m_tReadings[i].Color);
         CLuaUtility::AddToTable(pt_lua_state, "horizontal_idx", m_tReadings[i].HorizontalIndex);
         CLuaUtility::AddToTable(pt_lua_state, "vertical_idx", m_tReadings[i].VerticalIndex);
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
