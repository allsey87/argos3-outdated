/**
 * @file <argos3/plugins/robot/prototype/control_interface/ci_prototype_colored_blob_forwards_camera_sensor.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "ci_prototype_forwards_camera_sensor.h"

#ifdef ARGOS_WITH_LUA
#include <argos3/core/wrappers/lua/lua_utility.h>
#endif

namespace argos {

   /****************************************/
   /****************************************/

#ifdef ARGOS_WITH_LUA
   int LuaEnableCamera(lua_State* pt_lua_state) {
      /* Perform action */
      CLuaUtility::GetDeviceInstance<CCI_PrototypeForwardsCameraSensor>(pt_lua_state, "prototype_forwards_camera")->Enable();
         return 0;
   }
#endif

   /****************************************/
   /****************************************/

#ifdef ARGOS_WITH_LUA
   int LuaDisableCamera(lua_State* pt_lua_state) {
      /* Perform action */
      CLuaUtility::GetDeviceInstance<CCI_PrototypeForwardsCameraSensor>(pt_lua_state, "prototype_forwards_camera")->Disable();
         return 0;
   }
#endif

   /****************************************/
   /****************************************/

#ifdef ARGOS_WITH_LUA
   void CCI_PrototypeForwardsCameraSensor::CreateLuaState(lua_State* pt_lua_state) {
      CLuaUtility::OpenRobotStateTable(pt_lua_state, "prototype_forwards_camera");
      CLuaUtility::AddToTable(pt_lua_state, "_instance", this);
      //CLuaUtility::AddToTable(pt_lua_state, "enable", &LuaEnableCamera);
      //CLuaUtility::AddToTable(pt_lua_state, "disable", &LuaDisableCamera);
      for(size_t i = 0; i < m_tReadings.size(); i++) {
         CLuaUtility::StartTable(pt_lua_state,  i+1);
         for(size_t j = 0; j < m_tReadings[i].ObservationList.size(); ++j) {
            SObservation& sObservation = m_tReadings[i].ObservationList[j];
            CLuaUtility::StartTable(pt_lua_state, j+1);
            CLuaUtility::AddToTable(pt_lua_state, "color", sObservation.Color);
            CLuaUtility::AddToTable(pt_lua_state, "horizontal_idx", sObservation.HorizontalIndex);
            CLuaUtility::AddToTable(pt_lua_state, "vertical_idx", sObservation.VerticalIndex);
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
   void CCI_PrototypeForwardsCameraSensor::ReadingsToLuaState(lua_State* pt_lua_state) {
      lua_getfield(pt_lua_state, -1, "prototype_forwards_camera");
      /* Save the number of elements in the blob list */
      size_t unLastObservationNum = lua_objlen(pt_lua_state, -1);
      /* Overwrite the table with the new messages */
      for(size_t i = 0; i < m_tReadings.size(); i++) {
         CLuaUtility::StartTable(pt_lua_state, m_tDescriptors[i].Id);
         for(size_t j = 0; j < m_tReadings[i].ObservationList.size(); ++j) {
            SObservation& sObservation = m_tReadings[i].ObservationList[j];
            CLuaUtility::StartTable(pt_lua_state, j+1);
            CLuaUtility::AddToTable(pt_lua_state, "color", sObservation.Color);
            CLuaUtility::AddToTable(pt_lua_state, "horizontal_idx", sObservation.HorizontalIndex);
            CLuaUtility::AddToTable(pt_lua_state, "vertical_idx", sObservation.VerticalIndex);
            CLuaUtility::EndTable(pt_lua_state);
         }
         CLuaUtility::EndTable(pt_lua_state);
      
         /* Are the new messages less than the old ones? */
         if(m_tReadings[i].ObservationList.size() < unLastObservationNum) {
            /* Yes, set to nil all the extra entries */
            for(size_t j = m_tReadings[i].ObservationList.size() + 1; j <= unLastObservationNum; ++j) {
               lua_pushnumber(pt_lua_state,  j);
               lua_pushnil   (pt_lua_state    );
               lua_settable  (pt_lua_state, -3);
            }
         }
      }
      lua_pop(pt_lua_state, 1);
   }
#endif

   /****************************************/
   /****************************************/

}
