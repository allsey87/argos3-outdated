/**
 * @file <argos3/plugins/robots/prototype/simulator/entities/tag_entity.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "tag_entity.h"
#include <argos3/core/simulator/space/space.h>
#include <argos3/plugins/robots/prototype/simulator/media/tag_medium.h>

namespace argos {

   /****************************************/
   /****************************************/

   CTagEntity::CTagEntity(CComposableEntity* pc_parent) :
      CPositionalEntity(pc_parent) {}

   /****************************************/
   /****************************************/

   CTagEntity::CTagEntity(CComposableEntity* pc_parent,
                                    const std::string& str_id,
                                    const std::string& str_payload,
                                    bool b_localizable,
                                    Real f_side_length) :
      CPositionalEntity(pc_parent, str_id, CVector3(), CQuaternion()),
      m_strPayload(str_payload),
      m_bLocalizable(b_localizable),
      m_fSideLength(f_side_length) {}

   /****************************************/
   /****************************************/

   void CTagEntity::Init(TConfigurationNode& t_tree) {
      try {
         /* Parse XML */
         CPositionalEntity::Init(t_tree);
         GetNodeAttribute(t_tree, "localizable", m_bLocalizable);
         GetNodeAttribute(t_tree, "side_length", m_fSideLength);
         std::string strPayload;
         GetNodeAttribute(t_tree, "payload", m_strPayload);
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Error while initializing tag entity", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CTagEntity::AddToMedium(CTagMedium& c_medium) {
      c_medium.AddEntity(*this);
   }

   /****************************************/
   /****************************************/

   void CTagEntity::RemoveFromMedium(CTagMedium& c_medium) {
      c_medium.RemoveEntity(*this);
   }

   /****************************************/
   /****************************************/

   void CTagEntitySpaceHashUpdater::operator()(CAbstractSpaceHash<CTagEntity>& c_space_hash,
                                                    CTagEntity& c_element) {
      /* Calculate the position of the tag in the space hash */
      c_space_hash.SpaceToHashTable(m_nI, m_nJ, m_nK, c_element.GetPosition());
      /* Update the corresponding cell */
      c_space_hash.UpdateCell(m_nI, m_nJ, m_nK, c_element);
   }

   /****************************************/
   /****************************************/

   CTagEntityGridUpdater::CTagEntityGridUpdater(CGrid<CTagEntity>& c_grid) :
      m_cGrid(c_grid) {}

   /****************************************/
   /****************************************/

   bool CTagEntityGridUpdater::operator()(CTagEntity& c_entity) {
      try {
         /* Calculate the position of the tag in the space hash */
         m_cGrid.PositionToCell(m_nI, m_nJ, m_nK, c_entity.GetPosition());
         /* Update the corresponding cell */
         m_cGrid.UpdateCell(m_nI, m_nJ, m_nK, c_entity);
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("While updating the tag grid for tag \"" << c_entity.GetContext() << c_entity.GetId() << "\"", ex);
      }
      /* Continue with the other entities */
      return true;
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_SPACE_OPERATIONS_ON_ENTITY(CTagEntity);

   /****************************************/
   /****************************************/

}
