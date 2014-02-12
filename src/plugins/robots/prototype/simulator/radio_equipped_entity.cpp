/**
 * @file <argos3/plugins/robots/prototype/simulator/radio_equipped_entity.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "radio_equipped_entity.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/plugins/robots/prototype/simulator/radio_medium.h>
#include <argos3/plugins/robots/prototype/simulator/body_entity.h>

namespace argos {

   /****************************************/
   /****************************************/

   CRadioEquippedEntity::CRadioEquippedEntity(CComposableEntity* pc_parent) :
      CComposableEntity(pc_parent) {
   }

   /****************************************/
   /****************************************/

   CRadioEquippedEntity::CRadioEquippedEntity(CComposableEntity* pc_parent,
                                                    const std::string& str_id) :
      CComposableEntity(pc_parent, str_id) {
   }

   /****************************************/
   /****************************************/

   void CRadioEquippedEntity::Init(TConfigurationNode& t_tree) {
      try {
         /* Init parent */
         CComposableEntity::Init(t_tree);
         /* Go through the radio entries */
         TConfigurationNodeIterator itRadio("radio");
         for(itRadio = itRadio.begin(&t_tree);
             itRadio != itRadio.end();
             ++itRadio) {
            /* Initialise the radio using the XML */
            CRadioEntity* pcRadio = new CRadioEntity(this);
            pcRadio->Init(*itRadio);
            /* Add the Radio to this container */
            std::string strRadioBody;
            GetNodeAttribute(*itRadio, "body", strRadioBody);
            CBodyEntity& cRadioBody = GetParent().GetComponent<CBodyEntity>("bodies.body[" + strRadioBody + "]");
            CVector3 cPositionOffset;
            GetNodeAttribute(*itRadio, "position", cPositionOffset);
            CQuaternion cOrientationOffset;
            GetNodeAttribute(*itRadio, "orientation", cOrientationOffset);

            m_vecPositionalEntities.push_back(&cRadioBody.GetPositionalEntity());
            m_vecPositionOffsets.push_back(cPositionOffset);
            m_vecOrientationOffsets.push_back(cOrientationOffset);

            m_tRadios.push_back(pcRadio);
            AddComponent(*pcRadio);
         }
         UpdateComponents();
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Failed to initialize radio equipped entity \"" << GetId() << "\".", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CRadioEquippedEntity::Reset() {
      for(CRadioEntity::TList::iterator itRadio = m_tRadios.begin();
          itRadio != m_tRadios.end();
          ++itRadio) {
         (*itRadio)->Reset();
      }
   }

   /****************************************/
   /****************************************/

   void CRadioEquippedEntity::Update() {
      /* Set Radio position wrt reference */
      CVector3 cRadioPosition;
      CQuaternion cRadioOrientation;
      for(UInt32 i = 0; i < m_tRadios.size(); ++i) {
         cRadioPosition = m_vecPositionOffsets[i];
         cRadioPosition.Rotate(m_vecPositionalEntities[i]->GetOrientation());
         cRadioPosition += m_vecPositionalEntities[i]->GetPosition();

         cRadioOrientation = m_vecPositionalEntities[i]->GetOrientation() * m_vecOrientationOffsets[i];

         m_tRadios[i]->SetPosition(cRadioPosition);
         m_tRadios[i]->SetOrientation(cRadioOrientation);
      }
   }
   
   /****************************************/
   /****************************************/

   CRadioEntity& CRadioEquippedEntity::GetRadio(UInt32 un_index) {
      ARGOS_ASSERT(un_index < m_tRadios.size(),
                   "CRadioEquippedEntity::GetRadio(), id=\"" <<
                   GetContext() + GetId() <<
                   "\": index out of bounds: un_index = " <<
                   un_index <<
                   ", m_tRadios.size() = " <<
                   m_tRadios.size());
      return *m_tRadios[un_index];
   }

   /****************************************/
   /****************************************/

   const CQuaternion& CRadioEquippedEntity::GetOffsetOrientation(UInt32 un_index) const {
      ARGOS_ASSERT(un_index < m_vecOrientationOffsets.size(),
                   "CRadioEquippedEntity::GetOffsetOrientation(), id=\"" <<
                   GetContext() + GetId() <<
                   "\": index out of bounds: un_index = " <<
                   un_index <<
                   ", m_vecOrientationOffsets.size() = " <<
                   m_vecOrientationOffsets.size());
      return m_vecOrientationOffsets[un_index];
   }

   /****************************************/
   /****************************************/
   
   const CVector3& CRadioEquippedEntity::GetOffsetPosition(UInt32 un_index) const {
      ARGOS_ASSERT(un_index < m_vecPositionOffsets.size(),
                   "CRadioEquippedEntity::GetOffsetPosition(), id=\"" <<
                   GetContext() + GetId() <<
                   "\": index out of bounds: un_index = " <<
                   un_index <<
                   ", m_vecPositionOffsets.size() = " <<
                   m_vecPositionOffsets.size());
      return m_vecPositionOffsets[un_index];
   }

   /****************************************/
   /****************************************/
   
   const CPositionalEntity& CRadioEquippedEntity::GetPositionalEntity(UInt32 un_index) const {
      ARGOS_ASSERT(un_index < m_vecPositionalEntities.size(),
                   "CRadioEquippedEntity::GetPositionalEntity(), id=\"" <<
                   GetContext() + GetId() <<
                   "\": index out of bounds: un_index = " <<
                   un_index <<
                   ", m_vecPositionalEntities.size() = " <<
                   m_vecPositionalEntities.size());
      return *m_vecPositionalEntities[un_index];
   }

   /****************************************/
   /****************************************/

   void CRadioEquippedEntity::AddToMedium(CRadioMedium& c_medium) {
      for(UInt32 i = 0; i < m_tRadios.size(); ++i) {
         m_tRadios[i]->AddToMedium(c_medium);
      }
   }

   /****************************************/
   /****************************************/

   void CRadioEquippedEntity::RemoveFromMedium(CRadioMedium& c_medium) {
      for(UInt32 i = 0; i < m_tRadios.size(); ++i) {
         m_tRadios[i]->RemoveFromMedium(c_medium);
      }
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_SPACE_OPERATIONS_ON_COMPOSABLE(CRadioEquippedEntity);

   /****************************************/
   /****************************************/

}
