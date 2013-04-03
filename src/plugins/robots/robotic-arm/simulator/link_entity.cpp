/**
 * @file <argos3/plugins/robots/robotic-arm/simulator/link_entity.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "link_entity.h"

namespace argos {

   /****************************************/
   /****************************************/

   CLinkEntity::CLinkEntity(CComposableEntity* pc_parent) :
      CComposableEntity(pc_parent) {}

   /****************************************/
   /****************************************/

   CLinkEntity::CLinkEntity(CComposableEntity* pc_parent,
                          const std::string& str_id,
                          const CVector3& c_position,
                          const CQuaternion& c_orientation) :
      CComposableEntity(pc_parent, str_id) {
      
      m_pcPositionalEntity = new CPositionalEntity(this, str_id, c_position, c_orientation);
      AddComponent(*m_pcPositionalEntity);
   }

   /****************************************/
   /****************************************/

   void CLinkEntity::Init(TConfigurationNode& t_tree) {
      try {
         /* Init parent */
         CComposableEntity::Init(t_tree);

         /* Parse XML */
         m_pcPositionalEntity = new CPositionalEntity(this);
         m_pcPositionalEntity->Init(t_tree);
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Error while initializing link entity", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CLinkEntity::Reset() {
      CComposableEntity::Reset();
   }

   /****************************************/
   /****************************************/

   void CLinkEntity::Destroy() {
      CComposableEntity::Destroy();
   }

   /****************************************/
   /****************************************/

   void CLinkEntity::UpdateComponents() {}

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_SPACE_OPERATIONS_ON_COMPOSABLE(CLinkEntity);

   /****************************************/
   /****************************************/

}
