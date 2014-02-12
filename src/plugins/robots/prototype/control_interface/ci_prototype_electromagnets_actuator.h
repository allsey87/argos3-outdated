/**
 * @file <argos3/plugins/robots/prototype/control_interface/ci_prototype_electromagnet_actuator.h>
 *
 * @brief This file provides the common interface definition of the electromagnet actuator.
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef CCI_PROTOTYPE_ELECTROMAGNET_ACTUATOR_H
#define CCI_PROTOTYPE_ELECTROMAGNET_ACTUATOR_H

namespace argos {
   class CCI_PrototypeElectromagnetsActuator;
}

#include <argos3/core/control_interface/ci_actuator.h>
#include <argos3/core/utility/math/vector3.h>

namespace argos {

   class CCI_PrototypeElectromagnetsActuator : virtual public CCI_Actuator {

   public:

      struct SConfiguration {
         /* Current */
         Real Current; 
         /**
          * Constructor
          */
         SConfiguration() :
            Current(0.0f) {}
         /**
          * Constructor with parameters
          * @param f_current The current
          */
         SConfiguration(Real f_current) :
            Current(f_current) {}
         /**
          * Vector of configurations
          */
         typedef std::vector<SConfiguration> TList;
      };

      struct SDescriptor {
         std::string Id;
         CVector3 PassiveField;
         CVector3 ActiveField;
         bool Enabled;

         /**
          * Constructor
          */
         SDescriptor() : 
            Id(""),
            PassiveField(CVector3::ZERO),
            ActiveField(CVector3::ZERO),
            Enabled(false) {}
         /**
          * Constructor with parameters
          * @param str_id Electromagnet Indentifier
          * @param c_passive_field Passive field of electromanget
          * @param c_active_field Active field of electromagnet
          * @param b_enabled Electromagnet Enabled
          */
         SDescriptor(std::string str_id,
                     const CVector3& c_passive_field,
                     const CVector3& c_active_field,
                     bool b_enabled) : 
            Id(str_id),
            PassiveField(c_passive_field),
            ActiveField(c_active_field),
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
      CCI_PrototypeElectromagnetsActuator() {
      }
      /**
       * Destructor.
       */
      virtual ~CCI_PrototypeElectromagnetsActuator() {}

      /**
       * Returns a reference to the current electromagnet configurations.
       * @return A reference to the current electromagnet configurations.
       */
      inline SConfiguration::TList& GetConfigurations() {
         return m_tConfigurations;
      }
      
      /**
       * Returns a reference to the electromagnet descriptors
       * @return A reference to the electromagnet descriptors
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
