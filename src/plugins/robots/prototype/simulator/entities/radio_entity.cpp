/**
 * @file <argos3/plugins/robots/prototype/simulator/entities/radio_entity.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "radio_entity.h"
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/plugins/robots/prototype/simulator/media/radio_medium.h>

namespace argos {



   /****************************************/
   /****************************************/

   CRadioEntity::CRadioEntity(CComposableEntity* pc_parent) :
      CPositionalEntity(pc_parent) {}

   /****************************************/
   /****************************************/

   CRadioEntity::CRadioEntity(CComposableEntity* pc_parent,
                              const std::string& str_id,
                              Real f_tx_range) :
      CPositionalEntity(pc_parent, str_id, CVector3(), CQuaternion()),
      m_fTxRange(f_tx_range) {}

   /****************************************/
   /****************************************/

   void CRadioEntity::Init(TConfigurationNode& t_tree) {
      try {
         /* Parse XML */
         CPositionalEntity::Init(t_tree);
         GetNodeAttribute(t_tree, "range", m_fTxRange);
         
         std::string strMedium;
         GetNodeAttribute(t_tree, "medium", strMedium);
         CRadioMedium* pcRadioMedium = &CSimulator::GetInstance().GetMedium<CRadioMedium>(strMedium);
         pcRadioMedium->AddEntity(*this);

      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Error while initializing radio entity", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CRadioEntity::AddToMedium(CRadioMedium& c_medium) {
      c_medium.AddEntity(*this);
   }

   /****************************************/
   /****************************************/

   void CRadioEntity::RemoveFromMedium(CRadioMedium& c_medium) {
      c_medium.RemoveEntity(*this);
   }

   /****************************************/
   /****************************************/

   void CRadioEntitySpaceHashUpdater::operator()(CAbstractSpaceHash<CRadioEntity>& c_space_hash,
                                                 CRadioEntity& c_element) {
      /* Calculate the position of the radio in the space hash */
      c_space_hash.SpaceToHashTable(m_nI, m_nJ, m_nK, c_element.GetPosition());
      /* Update the corresponding cell */
      c_space_hash.UpdateCell(m_nI, m_nJ, m_nK, c_element);
   }

   /****************************************/
   /****************************************/

   CRadioEntityGridUpdater::CRadioEntityGridUpdater(CGrid<CRadioEntity>& c_grid) :
      m_cGrid(c_grid) {}

   /****************************************/
   /****************************************/

   bool CRadioEntityGridUpdater::operator()(CRadioEntity& c_entity) {
      try {
         /* Calculate the position of the radio in the space hash */
         m_cGrid.PositionToCell(m_nI, m_nJ, m_nK, c_entity.GetPosition());
         /* Update the corresponding cell */
         m_cGrid.UpdateCell(m_nI, m_nJ, m_nK, c_entity);
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("While updating the grid for radio \"" << c_entity.GetContext() << c_entity.GetId() << "\"", ex);
      }
      /* Continue with the other entities */
      return true;
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_SPACE_OPERATIONS_ON_ENTITY(CRadioEntity);

   /****************************************/
   /****************************************/

}
