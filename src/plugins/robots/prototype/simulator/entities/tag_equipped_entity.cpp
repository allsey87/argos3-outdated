/**
 * @file <argos3/plugins/robots/prototype/simulator/entities/tag_equipped_entity.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "tag_equipped_entity.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/plugins/robots/prototype/simulator/media/tag_medium.h>
#include <argos3/plugins/robots/prototype/simulator/entities/body_entity.h>

namespace argos {

   /****************************************/
   /****************************************/

   CTagEquippedEntity::CTagEquippedEntity(CComposableEntity* pc_parent) :
      CComposableEntity(pc_parent) {
   }

   /****************************************/
   /****************************************/

   CTagEquippedEntity::CTagEquippedEntity(CComposableEntity* pc_parent,
                                          const std::string& str_id) :
      CComposableEntity(pc_parent, str_id) {
   }

   /****************************************/
   /****************************************/

   void CTagEquippedEntity::Init(TConfigurationNode& t_tree) {
      try {
         /* Init parent */
         CComposableEntity::Init(t_tree);
         /* Go through the tag entries */
         TConfigurationNodeIterator itTag("tag");
         for(itTag = itTag.begin(&t_tree);
             itTag != itTag.end();
             ++itTag) {
            /* Initialise the tag using the XML */
            CTagEntity* pcTag = new CTagEntity(this);
            pcTag->Init(*itTag);
            /* Add the tag to this container */
            std::string strTagBody;
            GetNodeAttribute(*itTag, "body", strTagBody);
            CBodyEntity& cTagBody = GetParent().GetComponent<CBodyEntity>("bodies.body[" + strTagBody + "]");
            /* store the offset position and orientation */
            CVector3 cPositionOffset;
            GetNodeAttribute(*itTag, "position", cPositionOffset);
            CQuaternion cOrientationOffset;
            GetNodeAttribute(*itTag, "orientation", cOrientationOffset);
            m_vecPositionalEntities.push_back(&cTagBody.GetPositionalEntity());
            m_vecPositionOffsets.push_back(cPositionOffset);
            m_vecOrientationOffsets.push_back(cOrientationOffset);
            m_tTags.push_back(pcTag);
            AddComponent(*pcTag);
         }
         UpdateComponents();
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Failed to initialize tag equipped entity \"" << GetId() << "\".", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CTagEquippedEntity::Reset() {
      for(CTagEntity::TList::iterator itTag = m_tTags.begin();
          itTag != m_tTags.end();
          ++itTag) {
         (*itTag)->Reset();
      }
   }

   /****************************************/
   /****************************************/

   void CTagEquippedEntity::Update() {
      /* Set tag position w.r.t reference */
      CVector3 cTagPosition;
      CQuaternion cTagOrientation;
      for(UInt32 i = 0; i < m_tTags.size(); ++i) {
         cTagPosition = m_vecPositionOffsets[i];
         cTagPosition.Rotate(m_vecPositionalEntities[i]->GetOrientation());
         cTagPosition += m_vecPositionalEntities[i]->GetPosition();
         cTagOrientation = m_vecPositionalEntities[i]->GetOrientation() *
            m_vecOrientationOffsets[i];
         m_tTags[i]->SetPosition(cTagPosition);
         m_tTags[i]->SetOrientation(cTagOrientation);
      }
   }
   
   /****************************************/
   /****************************************/

   CTagEntity& CTagEquippedEntity::GetTag(UInt32 un_index) {
      ARGOS_ASSERT(un_index < m_tTags.size(),
                   "CTagEquippedEntity::GetTag(), id=\"" <<
                   GetContext() + GetId() <<
                   "\": index out of bounds: un_index = " <<
                   un_index <<
                   ", m_tTags.size() = " <<
                   m_tTags.size());
      return *m_tTags[un_index];
   }

   /****************************************/
   /****************************************/

   const CQuaternion& CTagEquippedEntity::GetOffsetOrientation(UInt32 un_index) const {
      ARGOS_ASSERT(un_index < m_vecOrientationOffsets.size(),
                   "CTagEquippedEntity::GetOffsetOrientation(), id=\"" <<
                   GetContext() + GetId() <<
                   "\": index out of bounds: un_index = " <<
                   un_index <<
                   ", m_vecOrientationOffsets.size() = " <<
                   m_vecOrientationOffsets.size());
      return m_vecOrientationOffsets[un_index];
   }

   /****************************************/
   /****************************************/
   
   const CVector3& CTagEquippedEntity::GetOffsetPosition(UInt32 un_index) const {
      ARGOS_ASSERT(un_index < m_vecPositionOffsets.size(),
                   "CTagEquippedEntity::GetOffsetPosition(), id=\"" <<
                   GetContext() + GetId() <<
                   "\": index out of bounds: un_index = " <<
                   un_index <<
                   ", m_vecPositionOffsets.size() = " <<
                   m_vecPositionOffsets.size());
      return m_vecPositionOffsets[un_index];
   }

   /****************************************/
   /****************************************/
   
   const CPositionalEntity& CTagEquippedEntity::GetPositionalEntity(UInt32 un_index) const {
      ARGOS_ASSERT(un_index < m_vecPositionalEntities.size(),
                   "CTagEquippedEntity::GetPositionalEntity(), id=\"" <<
                   GetContext() + GetId() <<
                   "\": index out of bounds: un_index = " <<
                   un_index <<
                   ", m_vecPositionalEntities.size() = " <<
                   m_vecPositionalEntities.size());
      return *m_vecPositionalEntities[un_index];
   }

   /****************************************/
   /****************************************/

   void CTagEquippedEntity::AddToMedium(CTagMedium& c_medium) {
      for(UInt32 i = 0; i < m_tTags.size(); ++i) {
         m_tTags[i]->AddToMedium(c_medium);
      }
   }

   /****************************************/
   /****************************************/

   void CTagEquippedEntity::RemoveFromMedium(CTagMedium& c_medium) {
      for(UInt32 i = 0; i < m_tTags.size(); ++i) {
         m_tTags[i]->RemoveFromMedium(c_medium);
      }
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_SPACE_OPERATIONS_ON_COMPOSABLE(CTagEquippedEntity);

   /****************************************/
   /****************************************/

}
