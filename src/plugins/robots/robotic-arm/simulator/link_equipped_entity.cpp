/**
 * @file <argos3/plugins/robots/robotic-arm/simulator/link_equipped_entity.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "link_equipped_entity.h"

namespace argos {

   /****************************************/
   /****************************************/

   CLinkEquippedEntity::CLinkEquippedEntity(CComposableEntity* pc_parent) :
      CComposableEntity(pc_parent) {}

   /****************************************/
   /****************************************/

   CLinkEquippedEntity::CLinkEquippedEntity(CComposableEntity* pc_parent,
                                          const std::string& str_id) :
      CComposableEntity(pc_parent, str_id) {}

   /****************************************/
   /****************************************/

   void CLinkEquippedEntity::Init(TConfigurationNode& t_tree) {
      try {
         /* Init parent */
         CComposableEntity::Init(t_tree);
         /* Go through the led entries */
         CVector3 cPosition;
         CQuaternion cOrientation;
         TConfigurationNodeIterator itLink("link");
         for(itLink = itLink.begin(&t_tree);
             itLink != itLink.end();
             ++itLink) {
            CLinkEntity* pcLink = new CLinkEntity(this);
            pcLink->Init(*itLink);
            m_tLinks.push_back(pcLink);
            AddComponent(*pcLink);
         }
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Failed to initialize link equipped entity \"" << GetId() << "\".", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CLinkEquippedEntity::Reset() {
      for(CLinkEntity::TList::iterator it = m_tLinks.begin();
          it != m_tLinks.end();
          ++it) {
         (*it)->Reset();
      }
   }

   /****************************************/
   /****************************************/

   void CLinkEquippedEntity::AddLink(const CVector3& c_position,
                                     const CQuaternion& c_orientation) {
      CLinkEntity* pcLink =
         new CLinkEntity(
            this,
            GetId() + ".link[" + ToString(m_tLinks.size()) + "]",
            c_position,
            c_orientation);
      m_tLinks.push_back(pcLink);
      AddComponent(*pcLink);
   }

   /****************************************/
   /****************************************/

   CLinkEntity& CLinkEquippedEntity::GetLink(UInt32 un_index) {
      ARGOS_ASSERT(un_index < m_tLinks.size(),
                   "CLinkEquippedEntity::GetLink(), id=\"" <<
                   GetId() <<
                   "\": index out of bounds: un_index = " <<
                   un_index <<
                   ", m_tLinks.size() = " <<
                   m_tLinks.size());
      return *m_tLinks[un_index];
   }

   /****************************************/
   /****************************************/

   void CLinkEquippedEntity::SetLinkPosition(UInt32 un_index,
                                             const CVector3& c_position) {
      ARGOS_ASSERT(un_index < m_tLinks.size(),
                   "CLinkEquippedEntity::SetLinkPosition(), id=\"" <<
                   GetId() <<
                   "\": index out of bounds: un_index = " <<
                   un_index <<
                   ", m_tLinks.size() = " <<
                   m_tLinks.size());
      m_tLinks[un_index]->GetPositionalEntity().SetPosition(c_position);
   }

   /****************************************/
   /****************************************/

   void CLinkEquippedEntity::SetLinkOrientation(UInt32 un_index,
                                                const CQuaternion& c_orientation) {
      ARGOS_ASSERT(un_index < m_tLinks.size(),
                   "CLinkEquippedEntity::SetLinkOrientation(), id=\"" <<
                   GetId() <<
                   "\": index out of bounds: un_index = " <<
                   un_index <<
                   ", m_tLinks.size() = " <<
                   m_tLinks.size());
      m_tLinks[un_index]->GetPositionalEntity().SetOrientation(c_orientation);
   }

   /****************************************/
   /****************************************/

   void CLinkEquippedEntity::UpdateComponents() {}

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_SPACE_OPERATIONS_ON_COMPOSABLE(CLinkEquippedEntity);

   /****************************************/
   /****************************************/

}
