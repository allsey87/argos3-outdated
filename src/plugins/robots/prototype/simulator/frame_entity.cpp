/**
 * @file <argos3/plugins/robots/prototype/simulator/frame_entity.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "frame_entity.h"

#include <argos3/plugins/robots/prototype/simulator/body_equipped_entity.h>

namespace argos {

   /****************************************/
   /****************************************/

   CFrameEntity::CFrameEntity(CComposableEntity* pc_parent) :
      CComposableEntity(pc_parent),
      m_pcBodyEntity(NULL),
      m_pcPositionalEntity(NULL) {}

   /****************************************/
   /****************************************/

   CFrameEntity::CFrameEntity(CComposableEntity* pc_parent,
                              const std::string& str_id,
                              CBodyEntity* pc_body,
                              const CVector3& c_position,
                              const CQuaternion& c_orientation) :
      CComposableEntity(pc_parent, str_id),
      m_pcBodyEntity(pc_body) {
      m_pcPositionalEntity = new CPositionalEntity(this, "offset", c_position, c_orientation);
      AddComponent(*m_pcPositionalEntity);
   }

   /****************************************/
   /****************************************/


   void CFrameEntity::Init(TConfigurationNode& t_tree) {
      try {
         /* Init parent */
         CComposableEntity::Init(t_tree);
         /* Init positional entity */
         m_pcPositionalEntity = new CPositionalEntity(this);
         m_pcPositionalEntity->Init(t_tree);
         AddComponent(*m_pcPositionalEntity);
         /* Create a link between this frame and the body in which it exists */
         std::string strBody;
         GetNodeAttribute(t_tree, "body", strBody);
         CComposableEntity* pcRootEntity = dynamic_cast<CComposableEntity*>(&GetRootEntity());
         if(pcRootEntity != NULL) {
            m_pcBodyEntity = &(pcRootEntity->GetComponent<CBodyEntity>("bodies.body[" + strBody + "]"));
         }
         else {
            THROW_ARGOSEXCEPTION("Failed to cast the root entity \"" << GetRootEntity().GetId() << "\" as a composable entity.");
         }
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Error while initializing frame entity", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CFrameEntity::Reset() {
      CComposableEntity::Reset();
   }

   /****************************************/
   /****************************************/

   void CFrameEntity::Destroy() {
      CComposableEntity::Destroy();
   }

   /****************************************/
   /****************************************/

   void CFrameEntity::UpdateComponents() {}

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_SPACE_OPERATIONS_ON_COMPOSABLE(CFrameEntity);

   /****************************************/
   /****************************************/

}
