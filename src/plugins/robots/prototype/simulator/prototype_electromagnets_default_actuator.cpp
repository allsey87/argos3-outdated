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
      std::string strTargets;
      std::vector<std::string> vecTargetIds;
      GetNodeAttribute(t_tree, "target", strTargets);
      Tokenize(strTargets, vecTargetIds, ",");
      GetNodeAttributeOrDefault(t_tree, "show_poles", m_bShowPoles, m_bShowPoles);
      /* Get references to the bodies and magnets */
      CBodyEntity::TList& tElectromagneticBodies = m_pcElectromagnetEquippedEntity->GetAllElectromagneticBodies();
      CElectromagnetEntity::TList& tElectromagnets = m_pcElectromagnetEquippedEntity->GetAllElectromagnets();
      /* Attempt to locate each electromagnet specified for actuation and populate the relevant data structures*/
      for(std::vector<std::string>::iterator itTargetId = vecTargetIds.begin();
          itTargetId != vecTargetIds.end();
          ++itTargetId) {
         UInt32 unElectromagnetIndex = 0;
         for(unElectromagnetIndex = 0; 
             unElectromagnetIndex < tElectromagneticBodies.size(); 
             ++unElectromagnetIndex) {
            if(tElectromagneticBodies[unElectromagnetIndex]->GetId() == *itTargetId) break;
         }
         if(unElectromagnetIndex < tElectromagneticBodies.size()) {
            m_vecElectromagnetIndices.push_back(unElectromagnetIndex);
            m_tDescriptors.push_back(SDescriptor(*itTargetId,
                                                 tElectromagnets[unElectromagnetIndex]->GetPassiveField(),
                                                 tElectromagnets[unElectromagnetIndex]->GetActiveField(),
                                                 true));
            m_tConfigurations.push_back(SConfiguration(0.0f));
         }
         else {
            THROW_ARGOSEXCEPTION("A electromagnetic body with Id=\"" <<
                                 *itTargetId << 
                                 "\" could not be found in \"" <<
                                 m_pcElectromagnetEquippedEntity->GetContext() + 
                                 m_pcElectromagnetEquippedEntity->GetId() <<
                                 "\".");
         }
      }   
   }
   
   
   /****************************************/
   /****************************************/

   void CPrototypeElectromagnetsDefaultActuator::SetRobot(CComposableEntity& c_entity) {
      try {
         m_pcControllableEntity = &(c_entity.GetComponent<CControllableEntity>("controller"));
         m_pcElectromagnetEquippedEntity = &(c_entity.GetComponent<CElectromagnetEquippedEntity>("electromagnets"));
         m_pcElectromagnetEquippedEntity->SetCanBeEnabledIfDisabled(true);
         m_pcElectromagnetEquippedEntity->Enable();
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
