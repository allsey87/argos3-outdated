/**
 * @file <argos3/plugins/robots/prototype/control_interface/ci_prototype_radios_actuator.h>
 *
 * @brief This file provides the common interface definition of the radio actuator.
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef CCI_PROTOTYPE_RADIOS_ACTUATOR_H
#define CCI_PROTOTYPE_RADIOS_ACTUATOR_H

namespace argos {
   class CCI_PrototypeRadiosActuator;
}

#include <argos3/core/control_interface/ci_actuator.h>
#include <argos3/core/utility/datatypes/byte_array.h>

namespace argos {

   /**
    * This radio implementation allows communication of packets from one to many robots that are
    * within a given transmission range. The implementation is very low level. Any concepts such
    * as throughput, addressing, encryption or formatting of the contents would be required to be
    * implemented in the controller.
    */
   class CCI_PrototypeRadiosActuator : virtual public CCI_Actuator {

   public:

      struct SConfiguration {
         std::vector<CByteArray> TxData; 
         /**
          * Constructor
          */
         SConfiguration() {}
         /**
          * Constructor with parameters
          * @param vec_tx_data The data
          */
         SConfiguration(const std::vector<CByteArray> vec_tx_data) :
            TxData(vec_tx_data) {}
         /**
          * Vector of configurations
          */
         typedef std::vector<SConfiguration> TList;
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
      CCI_PrototypeRadiosActuator() {}
      /**
       * Destructor.
       */
      virtual ~CCI_PrototypeRadiosActuator() {}

      /**
       * Returns a reference to the current radio configurations.
       * @return A reference to the current radio configurations.
       */
      inline SConfiguration::TList& GetConfigurations() {
         return m_tConfigurations;
      }
      
      /**
       * Returns a reference to the radio descriptors
       * @return A reference to the radio descriptors
       */
      virtual SDescriptor::TList& GetDescriptors() {
         return m_tDescriptors;
      }


#ifdef ARGOS_WITH_LUA
      virtual void CreateLuaState(lua_State*);
#endif

      protected:
         SDescriptor::TList m_tDescriptors;
         SConfiguration::TList m_tConfigurations;
         
   };
}

#endif
