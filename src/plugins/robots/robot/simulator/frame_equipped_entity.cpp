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
         /* Go through the frames */
         TConfigurationNodeIterator itFrame("frame");
         for(itFrame = itFrame.begin(&t_tree);
             itFrame != itFrame.end();
             ++itFrame) {
            
            CFrameEntity* pcFrame = new CFrameEntity(this);
            pcFrame->Init(*itFrame);
            AddComponent(*pcFrame);
         }     
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Failed to initialize frame equipped entity \"" << GetId() << "\".", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CFrameEquippedEntity::Reset() {
      for(CFrameEntity::TList::iterator itFrame = m_tFrames.begin();
          itFrame != m_tFrames.end();
          ++itFrame) {
         (*itFrame)->Reset();
      }
   }

   /****************************************/
   /****************************************/

   void CFrameEquippedEntity::AddFrame(CBodyEntity* pc_body,
                                       const CVector3& c_position,
                                       const CQuaternion& c_orientation) {
      CFrameEntity* pcFrame =
         new CFrameEntity(
            this,
            GetId() + ".frame_" + ToString(m_tFrames.size()),
            pc_body,
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
