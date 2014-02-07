/**
 * @file <argos3/plugins/robots/prototype/simulator/barcode2_entity.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "barcode2_entity.h"
#include <argos3/core/simulator/space/space.h>
#include <argos3/plugins/robots/prototype/simulator/barcode2_medium.h>

namespace argos {



   /****************************************/
   /****************************************/

   CBarcode2Entity::CBarcode2Entity(CComposableEntity* pc_parent) :
      CPositionalEntity(pc_parent) {}

   /****************************************/
   /****************************************/

   CBarcode2Entity::CBarcode2Entity(CComposableEntity* pc_parent,
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

   void CBarcode2Entity::Init(TConfigurationNode& t_tree) {
      try {
         /* Parse XML */
         CPositionalEntity::Init(t_tree);
         GetNodeAttribute(t_tree, "localizable", m_bLocalizable);

         GetNodeAttribute(t_tree, "side_length", m_fSideLength);

         std::string strPayload;
         GetNodeAttribute(t_tree, "payload", m_strPayload);
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Error while initializing barcode2 entity", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CBarcode2Entity::AddToMedium(CBarcode2Medium& c_medium) {
      c_medium.AddEntity(*this);
   }

   /****************************************/
   /****************************************/

   void CBarcode2Entity::RemoveFromMedium(CBarcode2Medium& c_medium) {
      c_medium.RemoveEntity(*this);
   }

   /****************************************/
   /****************************************/

   void CBarcode2EntitySpaceHashUpdater::operator()(CAbstractSpaceHash<CBarcode2Entity>& c_space_hash,
                                                    CBarcode2Entity& c_element) {
      /* Calculate the position of the 2D barcode in the space hash */
      c_space_hash.SpaceToHashTable(m_nI, m_nJ, m_nK, c_element.GetPosition());
      /* Update the corresponding cell */
      c_space_hash.UpdateCell(m_nI, m_nJ, m_nK, c_element);
   }

   /****************************************/
   /****************************************/

   CBarcode2EntityGridUpdater::CBarcode2EntityGridUpdater(CGrid<CBarcode2Entity>& c_grid) :
      m_cGrid(c_grid) {}

   /****************************************/
   /****************************************/

   bool CBarcode2EntityGridUpdater::operator()(CBarcode2Entity& c_entity) {
      try {
         /* Calculate the position of the 2D barcode in the space hash */
         m_cGrid.PositionToCell(m_nI, m_nJ, m_nK, c_entity.GetPosition());
         /* Update the corresponding cell */
         m_cGrid.UpdateCell(m_nI, m_nJ, m_nK, c_entity);
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("While updating the Barcode2 grid for 2D Barcode \"" << c_entity.GetContext() << c_entity.GetId() << "\"", ex);
      }
      /* Continue with the other entities */
      return true;
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_SPACE_OPERATIONS_ON_ENTITY(CBarcode2Entity);

   /****************************************/
   /****************************************/

}
