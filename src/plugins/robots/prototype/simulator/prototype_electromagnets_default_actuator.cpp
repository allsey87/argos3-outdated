/**
 * @file <argos3/plugins/robots/prototype/simulator/prototype_electromagnets_default_actuator.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "prototype_electromagnets_default_actuator.h"

namespace argos {

   /****************************************/
   /****************************************/

   CPrototypeElectromagnetsDefaultActuator::CPrototypeElectromagnetsDefaultActuator() :
      m_pcElectromagnetEquippedEntity(NULL),
      m_bShowPoles(false) {
   }

   /****************************************/
   /****************************************/

   void CPrototypeElectromagnetsDefaultActuator::Init(TConfigurationNode& t_tree) {
      std::string strTarget;
      GetNodeAttribute(t_tree, "target", strTarget);
      Tokenize(strTarget, m_vecElectromagneticBodyIds, ",");
      GetNodeAttributeOrDefault(t_tree, "show_poles", m_bShowPoles, m_bShowPoles);
   }
   

   /****************************************/
   /****************************************/

   void CPrototypeElectromagnetsDefaultActuator::SetRobot(CComposableEntity& c_entity) {
      try {
         m_pcControllableEntity = &(c_entity.GetComponent<CControllableEntity>("controller"));
         m_pcElectromagnetEquippedEntity = &(c_entity.GetComponent<CElectromagnetEquippedEntity>("electromagnets"));
         m_pcElectromagnetEquippedEntity->SetCanBeEnabledIfDisabled(true);
         m_pcElectromagnetEquippedEntity->Enable();
         /* Get references to the lists of magnets and their associated bodies */
         CBodyEntity::TList& tElectromagneticBodies = m_pcElectromagnetEquippedEntity->GetAllElectromagneticBodies();
         CElectromagnetEntity::TList& tElectromagnets = m_pcElectromagnetEquippedEntity->GetAllElectromagnets();
         /* Attempt to locate each electromagnet specified for actuation and populate the relevant data structures*/
         for(std::vector<std::string>::iterator itElectromagneticBodyId = m_vecElectromagneticBodyIds.begin();
             itElectromagneticBodyId != m_vecElectromagneticBodyIds.end();
             ++itElectromagneticBodyId) {
            UInt32 unElectromagnetIndex = 0;
            for(unElectromagnetIndex = 0; 
                unElectromagnetIndex < tElectromagneticBodies.size(); 
                ++unElectromagnetIndex) {
               if(tElectromagneticBodies[unElectromagnetIndex]->GetId() == *itElectromagneticBodyId) break;
            }
            if(unElectromagnetIndex < tElectromagneticBodies.size()) {
               m_vecElectromagnetIndices.push_back(unElectromagnetIndex);
               m_tDescriptors.push_back(SDescriptor(*itElectromagneticBodyId,
                                                    tElectromagnets[unElectromagnetIndex]->GetActiveField(),
                                                    tElectromagnets[unElectromagnetIndex]->GetPassiveField(),
                                                    true));
               m_tConfigurations.push_back(SConfiguration(0.0f));
            }
            else {
               THROW_ARGOSEXCEPTION("A electromagnetic body with Id=\"" <<
                                    *itElectromagneticBodyId << 
                                    "\" could not be found in robot \"" <<
                                    c_entity.GetId() <<
                                    "\".");
            }
         }
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Can't set robot for the electromagnets default actuator", ex);
      }
   }


   /****************************************/
   /****************************************/

   void CPrototypeElectromagnetsDefaultActuator::Update() {
      CElectromagnetEntity::TList& tElectromagnets =
         m_pcElectromagnetEquippedEntity->GetAllElectromagnets();
      for(UInt32 unIdx = 0; unIdx < m_vecElectromagnetIndices.size(); ++unIdx) {
         tElectromagnets[m_vecElectromagnetIndices[unIdx]]->SetField(m_tDescriptors[unIdx].PassiveField + 
                                                                     m_tDescriptors[unIdx].ActiveField *
                                                                     m_tConfigurations[unIdx].Current);
      }
   }

   /****************************************/
   /****************************************/

   void CPrototypeElectromagnetsDefaultActuator::Reset() {
      for(UInt32 unIdx = 0; unIdx < m_vecElectromagnetIndices.size(); ++unIdx) {
         m_tConfigurations[unIdx].Current = 0.0f;
      }
   }

   /****************************************/
   /****************************************/

   REGISTER_ACTUATOR(CPrototypeElectromagnetsDefaultActuator,
                     "electromagnets", "default",
                     "Michael Allwright [allsey87@gmail.com]",
                     "1.0",
                     "The prototype electromagnet actuator.",
                     "This actuator allows a specified magnet field to be offset via a current as if their\n"
                     "were a current passing through a coil. For a complete description of its usage, refer\n"
                     "to the ci_prototype_electromagnet_actuator file.\n\n"
                     "REQUIRED XML CONFIGURATION\n\n"
                     "  <controllers>\n"
                     "    ...\n"
                     "    <my_controller ...>\n"
                     "      ...\n"
                     "      <actuators>\n"
                     "        ...\n"
                     "        <electromagnets implementation=\"default\"/>\n"
                     "        ...\n"
                     "      </actuators>\n"
                     "      ...\n"
                     "    </my_controller>\n"
                     "    ...\n"
                     "  </controllers>\n\n"
                     "OPTIONAL XML CONFIGURATION\n\n"
                     "None for the time being.\n",
                     "Usable"
      );
}
