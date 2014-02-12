/**
 * @file <argos3/plugins/robot/prototype/control_interface/ci_prototype_radios_sensor.cpp>
 *
 * @brief This file provides the common interface definition of the radio sensor.
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef CI_PROTOTYPE_RADIOS_SENSOR_H
#define CI_PROTOTYPE_RADIOS_SENSOR_H

namespace argos {
	class CCI_PrototypeRadiosSensor;
}

#include <argos3/core/control_interface/ci_sensor.h>
#include <argos3/core/utility/datatypes/byte_array.h>

namespace argos {
   
   /**
    * This radio implementation allows communication of packets from one to many robots that are
    * within a given transmission range. The implementation is very low level. Any concepts such
    * as throughput, addressing, encryption or formatting of the contents would be required to be
    * implemented in the controller.
    */
   class CCI_PrototypeRadiosSensor: virtual public CCI_Sensor {
      
   public:
      
      struct SReading {
         std::vector<CByteArray> RxData;       
         /**
          * Constructor
          */
         SReading() {}
         /**
          * Constructor with parameters
          * @param vec_rx_data The inbound data
          */
         SReading(const std::vector<CByteArray> vec_rx_data) :
            RxData(vec_rx_data) {}
         /**
          * Vector of the readings.
          */
         typedef std::vector<SReading> TList;
      };
      
      struct SDescriptor {
         std::string Id;
         bool Enabled;
         /**
          * Constructor
          */
         SDescriptor() : 
            Id(""),
            Enabled(false) {}
         /**
          * Constructor with parameters
          * @param str_id Radio Indentifier
          * @param b_enabled Radio Enabled
          */
         SDescriptor(std::string str_id,
                     bool b_enabled) : 
            Id(str_id),
            Enabled(b_enabled) {}
         /**
          * Vector of descriptors.
          */
         typedef std::vector<SDescriptor> TList;
      };


   public:
      
      /**
       * Constructor
       */
      CCI_PrototypeRadiosSensor() {
      }
      
      /**
       * Destructor
       */
      virtual ~CCI_PrototypeRadiosSensor() {
      }
      
      /**
       * Returns a reference to the current radio readings.
       * @return A reference to the current radio readings.
       */
      inline const SReading::TList& GetReadings() const {
         return m_tReadings;
      }
      
      /**
       * Returns a reference to the radio descriptors
       * @return A reference to the radio descriptors
       */
      virtual const SDescriptor::TList& GetDescriptors() const {
         return m_tDescriptors;
      }
      
#ifdef ARGOS_WITH_LUA
      virtual void CreateLuaState(lua_State* pt_lua_state);
      
      virtual void ReadingsToLuaState(lua_State* pt_lua_state);
#endif

   protected:
 
      SReading::TList m_tReadings;
      SDescriptor::TList m_tDescriptors;
      
   };
   
}

#endif
