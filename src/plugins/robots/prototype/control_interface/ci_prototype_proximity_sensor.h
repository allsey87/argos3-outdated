/**
 * @file <argos3/plugins/robots/prototype/control_interface/ci_prototype_proximity_sensor.h>
 *
 * @brief This file provides the definition of the prototype entity's proximity sensor.
 *
 */

#ifndef CCI_PROTOTYPE_PROXIMITY_SENSOR_H
#define CCI_PROTOTYPE_PROXIMITY_SENSOR_H

namespace argos {
   class CCI_PrototypeProximitySensor;
}

#include <argos3/core/control_interface/ci_sensor.h>
#include <argos3/core/utility/math/vector3.h>

namespace argos {

   class CCI_PrototypeProximitySensor : public CCI_Sensor {

   public:

      struct SReading {
         Real Value;
         CVector3 SensorOffset;
         CVector3 SensorDirection;

         SReading() :
            Value(0.0f) {}

         SReading(Real f_value,
                  const CVector3& c_sensor_offset,
                  const CVector3& c_sensor_direction) :
            Value(f_value),
            SensorOffset(c_sensor_offset),
            SensorDirection(c_sensor_direction) {}
      };

      typedef std::vector<SReading> TReadings;

   public:

      /**
       * Class constructor
       */
      CCI_PrototypeProximitySensor() {}

      /**
       * Class destructor
       */
      virtual ~CCI_PrototypeProximitySensor() {}

      /**
       * Returns the readings of this sensor
       */
      inline const TReadings& GetReadings() const {
         return m_tReadings;
      }

#ifdef ARGOS_WITH_LUA
      virtual void CreateLuaState(lua_State* pt_lua_state);

      virtual void ReadingsToLuaState(lua_State* pt_lua_state);
#endif

   protected:

      TReadings m_tReadings;

   };
   /*
   std::ostream& operator<<(std::ostream& c_os, const CCI_PrototypeProximitySensor::SReading& s_reading);
   std::ostream& operator<<(std::ostream& c_os, const CCI_PrototypeProximitySensor::TReadings& t_readings);
   */
}

#endif
