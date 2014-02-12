/**
 * @file <argos3/plugins/robots/prototype/simulator/prototype_radios_default_actuator.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "prototype_radios_default_actuator.h"

#include <argos3/plugins/robots/prototype/simulator/radio_equipped_entity.h>

namespace argos {

   /****************************************/
   /****************************************/

   CPrototypeRadiosDefaultActuator::CPrototypeRadiosDefaultActuator() :
      m_pcRadioEquippedEntity(NULL),
      m_pcControllableEntity(NULL) {
   }

   /****************************************/
   /****************************************/

   void CPrototypeRadiosDefaultActuator::SetRobot(CComposableEntity& c_entity) {
      try {
         /* Get and enable omndirectional radio equipped entity */
         m_pcRadioEquippedEntity = &(c_entity.GetComponent<CRadioEquippedEntity>("radio_container"));
         m_pcRadioEquippedEntity->SetCanBeEnabledIfDisabled(true);
         /* Create a configuration settings for each radio in the container */
         m_tConfigurations.resize(m_pcRadioEquippedEntity->GetAllRadios().size());
         m_tDescriptors.resize(m_pcRadioEquippedEntity->GetAllRadios().size());
         /* Populate the descriptors */
         for(size_t i = 0; i < m_pcRadioEquippedEntity->GetAllRadios().size(); ++i) {
            m_tDescriptors[i] = SDescriptor(m_pcRadioEquippedEntity->GetRadio(i).GetId(),
                                            m_pcRadioEquippedEntity->GetRadio(i).IsEnabled());
         }
         /* Get controllable entity */
         m_pcControllableEntity = &(c_entity.GetComponent<CControllableEntity>("controller"));
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Can't set robot for the radios default actuator", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CPrototypeRadiosDefaultActuator::Init(TConfigurationNode& t_tree) {
      try {
         /* Parent class init */
         CCI_PrototypeRadiosActuator::Init(t_tree);
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Error initializing the radios default actuator", ex);
      }
   }

   
   /****************************************/
   /****************************************/

   void CPrototypeRadiosDefaultActuator::Update() {
      for(size_t i = 0; i < m_pcRadioEquippedEntity->GetAllRadios().size(); ++i) {
         /* Swap out the data for transmission */
         m_tConfigurations[i].TxData.swap(m_pcRadioEquippedEntity->GetRadio(i).GetTxData());
         /* Clear the swapped in buffer */
         m_tConfigurations[i].TxData.clear();
      }
   }

   /****************************************/
   /****************************************/

   void CPrototypeRadiosDefaultActuator::Reset() {
      for(size_t i = 0; i < m_pcRadioEquippedEntity->GetAllRadios().size(); ++i) {
         /* Clear the existing data */
         m_tConfigurations[i].TxData.clear();
      }
   }

   /****************************************/
   /****************************************/

   REGISTER_ACTUATOR(CPrototypeRadiosDefaultActuator,
                     "radios", "default",
                     "Michael Allwright [allsey87@gmail.com]",
                     "1.0",
                     "A generic radio sensor to recieve messages from nearby radio actuators.",
                     "TODO\n\n",
                     "Under development"
   );
}
