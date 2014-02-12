/**
 * @file <argos3/plugins/robot/prototype/control_interface/ci_prototype_radio_sensor.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "ci_prototype_radios_sensor.h"

#ifdef ARGOS_WITH_LUA
#include <argos3/core/wrappers/lua/lua_utility.h>
#endif

namespace argos {

   /****************************************/
   /****************************************/

#ifdef ARGOS_WITH_LUA
   void CCI_PrototypeRadiosSensor::CreateLuaState(lua_State* pt_lua_state) {
      CLuaUtility::OpenRobotStateTable(pt_lua_state, "radios");
      //      CLuaUtility::AddToTable(pt_lua_state, "_instance", this);
      // i = radios
      // j = messages
      // k = bytes

      for(size_t i = 0; i < m_tReadings.size(); i++) {
         CLuaUtility::StartTable(pt_lua_state, m_tDescriptors[i].Id);
         for(size_t j = 0; j < m_tReadings[i].RxData.size(); ++j) {
            CLuaUtility::StartTable(pt_lua_state, j+1);
            for(size_t k = 0; k < m_tReadings[i].RxData[j].Size(); ++k) {
               CLuaUtility::AddToTable(pt_lua_state, k+1, m_tReadings[i].RxData[j][k]);
            }
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
   void CCI_PrototypeRadiosSensor::ReadingsToLuaState(lua_State* pt_lua_state) {
      lua_getfield(pt_lua_state, -1, "radios");
      /* Overwrite the table with the new messages */
      for(size_t i = 0; i < m_tReadings.size(); i++) {
         CLuaUtility::StartTable(pt_lua_state, m_tDescriptors[i].Id);
         

         size_t unLastMessageCnt = lua_objlen(pt_lua_state, -1);
         for(size_t j = 0; j < m_tReadings[i].RxData.size(); ++j) {

            CLuaUtility::StartTable(pt_lua_state, j);
            size_t unLastByteCnt = lua_objlen(pt_lua_state, -1);
            for(size_t k = 0; k < m_tReadings[i].RxData[j].Size(); ++k) {
               CLuaUtility::AddToTable(pt_lua_state, k, m_tReadings[i].RxData[j][k]);
            }
            CLuaUtility::EndTable(pt_lua_state);
            /* Is there more bytes in the new messages than in the old ones? */
            if(m_tReadings[i].RxData[j].Size() < unLastByteCnt) {
               /* Yes, set to nil all the extra entries */
               for(size_t k = m_tReadings[i].RxData[j].Size() + 1; k <= unLastByteCnt; ++k) {
                  lua_pushnumber(pt_lua_state,  k);
                  lua_pushnil   (pt_lua_state    );
                  lua_settable  (pt_lua_state, -3);
               }
            }
         }
         CLuaUtility::EndTable(pt_lua_state);
      
         /* Are the new messages less than the old ones? */
         if(m_tReadings[i].RxData.size() < unLastMessageCnt) {
            /* Yes, set to nil all the extra entries */
            for(size_t j = m_tReadings[i].RxData.size() + 1; j <= unLastMessageCnt; ++j) {
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
