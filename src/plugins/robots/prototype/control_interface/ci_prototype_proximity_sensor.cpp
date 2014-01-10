/**
 * @file <argos3/plugins/robots/prototype/control_interface/ci_prototype_proximity_sensor.cpp>
 *
 * @author Michael Allwright <allsey87@gmail.com>
 */

#include "ci_prototype_proximity_sensor.h"
#include <argos3/core/utility/math/angles.h>

#ifdef ARGOS_WITH_LUA
#include <argos3/core/wrappers/lua/lua_utility.h>
#endif

namespace argos {

   /****************************************/
   /****************************************/

#ifdef ARGOS_WITH_LUA
   void CCI_PrototypeProximitySensor::CreateLuaState(lua_State* pt_lua_state) {
      CLuaUtility::OpenRobotStateTable(pt_lua_state, "proximity");
      for(size_t i = 0; i < GetReadings().size(); ++i) {
         CLuaUtility::StartTable(pt_lua_state, i+1                                          );
         CLuaUtility::AddToTable(pt_lua_state, "offset",     m_tReadings[i].SensorOffset    );
         CLuaUtility::AddToTable(pt_lua_state, "direction",  m_tReadings[i].SensorDirection );
         CLuaUtility::AddToTable(pt_lua_state, "value",      m_tReadings[i].Value           );
         CLuaUtility::EndTable  (pt_lua_state                                               );
      }
      CLuaUtility::CloseRobotStateTable(pt_lua_state);
   }
#endif

   /****************************************/
   /****************************************/

#ifdef ARGOS_WITH_LUA
   void CCI_PrototypeProximitySensor::ReadingsToLuaState(lua_State* pt_lua_state) {
      lua_getfield(pt_lua_state, -1, "proximity");
      for(size_t i = 0; i < GetReadings().size(); ++i) {
         lua_pushnumber(pt_lua_state, i+1                 );
         lua_gettable  (pt_lua_state, -2                  );
         lua_pushnumber(pt_lua_state, m_tReadings[i].Value);
         lua_setfield  (pt_lua_state, -2, "value"         );
         lua_pop(pt_lua_state, 1);
      }
      lua_pop(pt_lua_state, 1);
   }
#endif

   
   /****************************************/
   /****************************************/

   /*

   std::ostream& operator<<(std::ostream& c_os,
                            const CCI_FootBotProximitySensor::SReading& s_reading) {
      c_os << "Value=<" << s_reading.Value
           << ">, Angle=<" << s_reading.Angle << ">";
      return c_os;
   }
   */

   /****************************************/
   /****************************************/

   /*
   std::ostream& operator<<(std::ostream& c_os,
                            const CCI_FootBotProximitySensor::TReadings& t_readings) {
      if(! t_readings.empty()) {
         c_os << "{ " << t_readings[0].Value << " }";
         for(UInt32 i = 1; i < t_readings.size(); ++i) {
            c_os << " { " << t_readings[0].Value << " }";
         }
         c_os << std::endl;
      }
      return c_os;
   }
   */

   /****************************************/
   /****************************************/

}
