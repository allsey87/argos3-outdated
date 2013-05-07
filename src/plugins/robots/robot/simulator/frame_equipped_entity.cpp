/**
 * @file <argos3/plugins/robots/robot/simulator/frame_equipped_entity.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "frame_equipped_entity.h"

namespace argos {

   /****************************************/
   /****************************************/

   CFrameEquippedEntity::CFrameEquippedEntity(CComposableEntity* pc_parent) :
      CComposableEntity(pc_parent) {}

   /****************************************/
   /****************************************/

   CFrameEquippedEntity::CFrameEquippedEntity(CComposableEntity* pc_parent,
                                              const std::string& str_id) :
      CComposableEntity(pc_parent, str_id) {}

   /****************************************/
   /****************************************/

   void CFrameEquippedEntity::Init(TConfigurationNode& t_tree) {
      try {
         /* Init parent */
         CComposableEntity::Init(t_tree);

         /* @todo remove this method and search the simulated space directly */
         CBodyEquippedEntity& cBodyEquippedEntity = GetParent().GetParent().GetParent().GetComponent<CBodyEquippedEntity>("bodies")

         /* Go through the frames */
         TConfigurationNodeIterator itFrame("frame");
         for(itFrame = itFrame.begin(&t_tree);
             itFrame != itFrame.end();
             ++itFrame) {
            
            std::string strBody;
            CVector3 cFramePosition;
            CQuaternion cFrameOrientation;
            
            GetNodeAttribute(*itFrame, "body", strBody);
            GetNodeAttribute(*itFrame, "position", cFramePosition);
            GetNodeAttribute(*itFrame, "orientation", cFrameOrientation);

            // @todo implement method to find this body directly from the space
            CBodyEntity& cBody = cBodyEquippedEntity.GetBody(strBody);
            fprintf(stderr, "Adding frame for body: %s\n", strBody.c_str());
            AddFrame(&cBody, cFramePosition, cFrameOrientation);
         }     
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Failed to initialize frame equipped entity \"" << GetId() << "\".", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CFrameEquippedEntity::Reset() {
      for(CFrameEntity::TList::iterator it = m_tFrames.begin();
          it != m_tFrames.end();
          ++it) {
         (*it)->Reset();
      }
   }

   /****************************************/
   /****************************************/

   void CFrameEquippedEntity::AddFrame(CBodyEntty* pc_body,
                                       const CVector3& c_position,
                                       const CQuaternion& c_orientation) {
      CFrameEntity* pcFrame =
         new CFrameEntity(
            this,
            GetId() + ".frame[" + ToString(m_tFrames.size()) + "]",
            c_position,
            c_orientation);
      m_tFrames.push_back(pcFrame);
      AddComponent(*pcFrame);
   }

   /****************************************/
   /****************************************/

   CFrameEntity& CFrameEquippedEntity::GetFrame(UInt32 un_index) {
      ARGOS_ASSERT(un_index < m_tFrames.size(),
                   "CFrameEquippedEntity::GetFrame(), id=\"" <<
                   GetId() <<
                   "\": index out of bounds: un_index = " <<
                   un_index <<
                   ", m_tFrames.size() = " <<
                   m_tFrames.size());
      return *m_tFrames[un_index];
   }

   /****************************************/
   /****************************************/

   void CFrameEquippedEntity::UpdateComponents() {}

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_SPACE_OPERATIONS_ON_COMPOSABLE(CFrameEquippedEntity);

   /****************************************/
   /****************************************/

}
