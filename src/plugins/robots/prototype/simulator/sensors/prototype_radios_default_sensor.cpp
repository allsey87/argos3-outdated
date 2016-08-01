/**
 * @file <argos3/plugins/robots/prototype/simulator/sensors/prototype_radios_default_sensor.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "prototype_radios_default_sensor.h"

#include <argos3/plugins/robots/prototype/simulator/entities/radio_equipped_entity.h>

namespace argos {

   /****************************************/
   /****************************************/

   CPrototypeRadiosDefaultSensor::CPrototypeRadiosDefaultSensor() :
      m_pcRadioEquippedEntity(NULL),
      m_pcControllableEntity(NULL) {}

   /****************************************/
   /****************************************/

   void CPrototypeRadiosDefaultSensor::SetRobot(CComposableEntity& c_entity) {
      try {
         /* Get and enable omndirectional radio equipped entity */
         m_pcRadioEquippedEntity = &(c_entity.GetComponent<CRadioEquippedEntity>("radios"));
         m_pcRadioEquippedEntity->SetCanBeEnabledIfDisabled(true);
         /* Create a readings list for each radio in the container */
         m_tReadings.resize(m_pcRadioEquippedEntity->GetAllRadios().size());
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
         THROW_ARGOSEXCEPTION_NESTED("Can't set robot for the radios default sensor", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CPrototypeRadiosDefaultSensor::Init(TConfigurationNode& t_tree) {
      try {
         /* Parent class init */
         CCI_PrototypeRadiosSensor::Init(t_tree);
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Error initializing the radios default sensor", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CPrototypeRadiosDefaultSensor::Update() {
      for(size_t i = 0; i < m_pcRadioEquippedEntity->GetAllRadios().size(); ++i) {
         /* Clear the existing buffer */
         m_tReadings[i].RxData.clear();
         /* Swap in the latest readings */
         m_tReadings[i].RxData.swap(m_pcRadioEquippedEntity->GetRadio(i).GetRxData());
      }
   }

   /****************************************/
   /****************************************/

   void CPrototypeRadiosDefaultSensor::Reset() {
      for(size_t i = 0; i < m_pcRadioEquippedEntity->GetAllRadios().size(); ++i) {
         /* Clear the existing data */
         m_tReadings[i].RxData.clear();
      }
   }

   /****************************************/
   /****************************************/

   REGISTER_SENSOR(CPrototypeRadiosDefaultSensor,
                   "radios", "default",
                   "Michael Allwright [allsey87@gmail.com]",
                   "1.0",
                   "A generic radio actuator to send messages to nearby radio sensors.",
                   "TODO\n\n",
                   "Under development"
   );

}
