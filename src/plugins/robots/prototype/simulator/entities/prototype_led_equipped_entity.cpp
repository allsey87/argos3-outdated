/**
 * @file <argos3/plugins/robots/prototype/simulator/entities/prototype_led_equipped_entity.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "prototype_led_equipped_entity.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/plugins/simulator/media/led_medium.h>
#include <argos3/plugins/robots/prototype/simulator/entities/body_entity.h>

namespace argos {

   /****************************************/
   /****************************************/

   CPrototypeLEDEquippedEntity::CPrototypeLEDEquippedEntity(CComposableEntity* pc_parent) :
      CComposableEntity(pc_parent) {
   }

   /****************************************/
   /****************************************/

   CPrototypeLEDEquippedEntity::CPrototypeLEDEquippedEntity(CComposableEntity* pc_parent,
                                                            const std::string& str_id) :
      CComposableEntity(pc_parent, str_id) {
   }

   /****************************************/
   /****************************************/

   void CPrototypeLEDEquippedEntity::Init(TConfigurationNode& t_tree) {
      try {
         /* Init parent */
         CComposableEntity::Init(t_tree);
         /* Go through the led entries */
         TConfigurationNodeIterator itLED("led");
         for(itLED = itLED.begin(&t_tree);
             itLED != itLED.end();
             ++itLED) {
            /* Initialise the LED using the XML */
            CLEDEntity* pcLED = new CLEDEntity(this);
            pcLED->Init(*itLED);
            /* Add the LED to this container */
            std::string strLEDBody;
            GetNodeAttribute(*itLED, "body", strLEDBody);
            CBodyEntity& cLEDBody = GetParent().GetComponent<CBodyEntity>("bodies.body[" + strLEDBody + "]");
            /* store the offset position and orientation */
            CVector3 cPositionOffset;
            GetNodeAttribute(*itLED, "position", cPositionOffset);
            CQuaternion cOrientationOffset;
            GetNodeAttribute(*itLED, "orientation", cOrientationOffset);
            m_vecPositionalEntities.push_back(&cLEDBody.GetPositionalEntity());
            m_vecPositionOffsets.push_back(cPositionOffset);
            m_vecOrientationOffsets.push_back(cOrientationOffset);
            m_tLEDs.push_back(pcLED);
            AddComponent(*pcLED);
         }
         UpdateComponents();
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Failed to initialize LED equipped entity \"" << GetId() << "\".", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CPrototypeLEDEquippedEntity::Reset() {
      for(CLEDEntity::TList::iterator itLED = m_tLEDs.begin();
          itLED != m_tLEDs.end();
          ++itLED) {
         (*itLED)->Reset();
      }
   }

   /****************************************/
   /****************************************/ 

   void CPrototypeLEDEquippedEntity::Update() {
      /* Set LED position w.r.t reference */
      CVector3 cLEDPosition;
      CQuaternion cLEDOrientation;
      for(UInt32 i = 0; i < m_tLEDs.size(); ++i) {
         cLEDPosition = m_vecPositionOffsets[i];
         cLEDPosition.Rotate(m_vecPositionalEntities[i]->GetOrientation());
         cLEDPosition += m_vecPositionalEntities[i]->GetPosition();
         cLEDOrientation = m_vecPositionalEntities[i]->GetOrientation() *
            m_vecOrientationOffsets[i];
         m_tLEDs[i]->SetPosition(cLEDPosition);
         m_tLEDs[i]->SetOrientation(cLEDOrientation);
      }
   }

   /****************************************/
   /****************************************/
   
   CLEDEntity& CPrototypeLEDEquippedEntity::GetLED(UInt32 un_index) {
      ARGOS_ASSERT(un_index < m_tLEDs.size(),
                   "CPrototypeLEDEquippedEntity::GetLED(), id=\"" <<
                   GetId() <<
                   "\": index out of bounds: un_index = " <<
                   un_index <<
                   ", m_tLEDs.size() = " <<
                   m_tLEDs.size());
      return *m_tLEDs[un_index];
   }

   /****************************************/
   /****************************************/

   const CQuaternion& CPrototypeLEDEquippedEntity::GetOffsetOrientation(UInt32 un_index) const {
      ARGOS_ASSERT(un_index < m_vecOrientationOffsets.size(),
                   "CPrototypeLEDEquippedEntity::GetOffsetOrientation(), id=\"" <<
                   GetContext() + GetId() <<
                   "\": index out of bounds: un_index = " <<
                   un_index <<
                   ", m_vecOrientationOffsets.size() = " <<
                   m_vecOrientationOffsets.size());
      return m_vecOrientationOffsets[un_index];
   }

   /****************************************/
   /****************************************/
   
   const CVector3& CPrototypeLEDEquippedEntity::GetOffsetPosition(UInt32 un_index) const {
      ARGOS_ASSERT(un_index < m_vecPositionOffsets.size(),
                   "CPrototypeLEDEquippedEntity::GetOffsetPosition(), id=\"" <<
                   GetContext() + GetId() <<
                   "\": index out of bounds: un_index = " <<
                   un_index <<
                   ", m_vecPositionOffsets.size() = " <<
                   m_vecPositionOffsets.size());
      return m_vecPositionOffsets[un_index];
   }

   /****************************************/
   /****************************************/
   
   const CPositionalEntity& CPrototypeLEDEquippedEntity::GetPositionalEntity(UInt32 un_index) const {
      ARGOS_ASSERT(un_index < m_vecPositionalEntities.size(),
                   "CPrototypeLEDEquippedEntity::GetPositionalEntity(), id=\"" <<
                   GetContext() + GetId() <<
                   "\": index out of bounds: un_index = " <<
                   un_index <<
                   ", m_vecPositionalEntities.size() = " <<
                   m_vecPositionalEntities.size());
      return *m_vecPositionalEntities[un_index];
   }
  
   /****************************************/
   /****************************************/

   void CPrototypeLEDEquippedEntity::SetLEDColor(UInt32 un_index,
                                                 const CColor& c_color) {
      ARGOS_ASSERT(un_index < m_tLEDs.size(),
                   "CPrototypeLEDEquippedEntity::SetLEDColor(), id=\"" <<
                   GetId() <<
                   "\": index out of bounds: un_index = " <<
                   un_index <<
                   ", m_tLEDs.size() = " <<
                   m_tLEDs.size());
      m_tLEDs[un_index]->SetColor(c_color);
   }

   /****************************************/
   /****************************************/

   void CPrototypeLEDEquippedEntity::SetAllLEDsColors(const CColor& c_color) {
      for(UInt32 i = 0; i < m_tLEDs.size(); ++i) {
         m_tLEDs[i]->SetColor(c_color);
      }
   }

   /****************************************/
   /****************************************/

   void CPrototypeLEDEquippedEntity::SetAllLEDsColors(const std::vector<CColor>& vec_colors) {
      if(vec_colors.size() == m_tLEDs.size()) {
         for(UInt32 i = 0; i < vec_colors.size(); ++i) {
            m_tLEDs[i]->SetColor(vec_colors[i]);
         }
      }
      else {
         THROW_ARGOSEXCEPTION(
            "CPrototypeLEDEquippedEntity::SetAllLEDsColors(), id=\"" <<
            GetId() <<
            "\": number of LEDs (" <<
            m_tLEDs.size() <<
            ") is lower than the passed color vector size (" <<
            vec_colors.size() <<
            ")");
      }
   }

   /****************************************/
   /****************************************/

   void CPrototypeLEDEquippedEntity::AddToMedium(CLEDMedium& c_medium) {
      for(UInt32 i = 0; i < m_tLEDs.size(); ++i) {
         m_tLEDs[i]->AddToMedium(c_medium);
      }
   }

   /****************************************/
   /****************************************/

   void CPrototypeLEDEquippedEntity::RemoveFromMedium(CLEDMedium& c_medium) {
      for(UInt32 i = 0; i < m_tLEDs.size(); ++i) {
         m_tLEDs[i]->RemoveFromMedium(c_medium);
      }
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_SPACE_OPERATIONS_ON_COMPOSABLE(CPrototypeLEDEquippedEntity);

   /****************************************/
   /****************************************/

}
