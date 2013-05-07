/**
 * @file <argos3/plugins/robots/robot/simulator/frame_entity.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "frame_entity.h"

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
                              CBodyEntty& c_body,
                              const CVector3& c_position,
                              const CQuaternion& c_orientation) :
      CComposableEntity(pc_parent, str_id),
      m_cBodyEntity(c_body);
      m_pcPositionalEntity = new CPositionalEntity(this, GetId() + ".offset", c_position, c_orientation);
      AddComponent(m_pcPositionalEntity);
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
         /* @todo: use identified string to locate the requested body */
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
