/**
 * @file <argos3/plugins/robots/prototype/simulator/entities/electromagnet_equipped_entity.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "electromagnet_equipped_entity.h"
#include <argos3/plugins/robots/prototype/simulator/entities/body_entity.h>

namespace argos {

   /****************************************/
   /****************************************/

   CElectromagnetEquippedEntity::CElectromagnetEquippedEntity(CComposableEntity* pc_parent) :
      CComposableEntity(pc_parent) {
   }

   /****************************************/
   /****************************************/

   CElectromagnetEquippedEntity::CElectromagnetEquippedEntity(CComposableEntity* pc_parent,
                                                              const std::string& str_id) :
      CComposableEntity(pc_parent, str_id) {
   }

   /****************************************/
   /****************************************/

   void CElectromagnetEquippedEntity::Init(TConfigurationNode& t_tree) {
      try {
         /* Init parent */
         CComposableEntity::Init(t_tree);
         /* Go through the led entries */
         TConfigurationNodeIterator itElectromagnet("electromagnet");
         for(itElectromagnet = itElectromagnet.begin(&t_tree);
             itElectromagnet != itElectromagnet.end();
             ++itElectromagnet) {
            /* Initialise the Electromagnet using the XML */
            CElectromagnetEntity* pcElectromagnet = new CElectromagnetEntity(this);
            pcElectromagnet->Init(*itElectromagnet);
            /* Add the Electromagnet to this container */
            std::string strElectromagnetBody;
            GetNodeAttribute(*itElectromagnet, "body", strElectromagnetBody);
            CBodyEntity& cElectromagnetBody = GetParent().GetComponent<CBodyEntity>("bodies.body[" + strElectromagnetBody + "]");
            m_tElectromagneticBodies.push_back(&cElectromagnetBody);
            m_tElectromagnets.push_back(pcElectromagnet);
            AddComponent(*pcElectromagnet);
         }
         UpdateComponents();
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Failed to initialize Electromagnet equipped entity \"" << GetId() << "\".", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CElectromagnetEquippedEntity::Reset() {
      for(CElectromagnetEntity::TList::iterator it = m_tElectromagnets.begin();
          it != m_tElectromagnets.end();
          ++it) {
         (*it)->Reset();
      }
   }

   /****************************************/
   /****************************************/

   CElectromagnetEntity& CElectromagnetEquippedEntity::GetElectromagnet(UInt32 un_index) {
      ARGOS_ASSERT(un_index < m_tElectromagnets.size(),
                   "CElectromagnetEquippedEntity::GetElectromagnet(), id=\"" <<
                   GetId() <<
                   "\": index out of bounds: un_index = " <<
                   un_index <<
                   ", m_tElectromagnets.size() = " <<
                   m_tElectromagnets.size());
      return *m_tElectromagnets[un_index];
   }
   
   /****************************************/
   /****************************************/
    
   CBodyEntity& CElectromagnetEquippedEntity::GetElectromagneticBody(UInt32 un_index) {
      ARGOS_ASSERT(un_index < m_tElectromagneticBodies.size(),
                   "CElectromagnetEquippedEntity::GetElectromagneticBody(), id=\"" <<
                   GetId() <<
                   "\": index out of bounds: un_index = " <<
                   un_index <<
                   ", m_tElectromagneticBodies.size() = " <<
                   m_tElectromagneticBodies.size());
      return *m_tElectromagneticBodies[un_index];
   
   }

   /****************************************/
   /****************************************/

   void CElectromagnetEquippedEntity::SetElectromagnetField(UInt32 un_index,
                                                            const CVector3& c_field) {
      ARGOS_ASSERT(un_index < m_tElectromagnets.size(),
                   "CElectromagnetEquippedEntity::SetElectromagnetField(), id=\"" <<
                   GetId() <<
                   "\": index out of bounds: un_index = " <<
                   un_index <<
                   ", m_tElectromagnets.size() = " <<
                   m_tElectromagnets.size());
      m_tElectromagnets[un_index]->SetField(c_field);
   }

   /****************************************/
   /****************************************/

   void CElectromagnetEquippedEntity::SetAllElectromagnetFields(const CVector3& c_field) {
      for(UInt32 i = 0; i < m_tElectromagnets.size(); ++i) {
         m_tElectromagnets[i]->SetField(c_field);
      }
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_SPACE_OPERATIONS_ON_COMPOSABLE(CElectromagnetEquippedEntity);

   /****************************************/
   /****************************************/

}
