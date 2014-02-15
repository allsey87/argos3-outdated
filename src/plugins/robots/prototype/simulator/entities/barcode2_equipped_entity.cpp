/**
 * @file <argos3/plugins/robots/prototype/simulator/entities/barcode2_equipped_entity.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "barcode2_equipped_entity.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/plugins/robots/prototype/simulator/media/barcode2_medium.h>
#include <argos3/plugins/robots/prototype/simulator/entities/body_entity.h>

namespace argos {

   /****************************************/
   /****************************************/

   CBarcode2EquippedEntity::CBarcode2EquippedEntity(CComposableEntity* pc_parent) :
      CComposableEntity(pc_parent) {
   }

   /****************************************/
   /****************************************/

   CBarcode2EquippedEntity::CBarcode2EquippedEntity(CComposableEntity* pc_parent,
                                                    const std::string& str_id) :
      CComposableEntity(pc_parent, str_id) {
   }

   /****************************************/
   /****************************************/

   void CBarcode2EquippedEntity::Init(TConfigurationNode& t_tree) {
      try {
         /* Init parent */
         CComposableEntity::Init(t_tree);
         /* Go through the 2d barcode entries */
         TConfigurationNodeIterator itBarcode("barcode");
         for(itBarcode = itBarcode.begin(&t_tree);
             itBarcode != itBarcode.end();
             ++itBarcode) {
            /* Initialise the Barcode2 using the XML */
            CBarcode2Entity* pcBarcode = new CBarcode2Entity(this);
            pcBarcode->Init(*itBarcode);
            /* Add the Barcode2 to this container */
            std::string strBarcodeBody;
            GetNodeAttribute(*itBarcode, "body", strBarcodeBody);
            CBodyEntity& cBarcodeBody = GetParent().GetComponent<CBodyEntity>("bodies.body[" + strBarcodeBody + "]");

            CVector3 cPositionOffset;
            GetNodeAttribute(*itBarcode, "position", cPositionOffset);
            CQuaternion cOrientationOffset;
            GetNodeAttribute(*itBarcode, "orientation", cOrientationOffset);

            m_vecPositionalEntities.push_back(&cBarcodeBody.GetPositionalEntity());
            m_vecPositionOffsets.push_back(cPositionOffset);
            m_vecOrientationOffsets.push_back(cOrientationOffset);

            m_tBarcodes.push_back(pcBarcode);
            AddComponent(*pcBarcode);
         }
         UpdateComponents();
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Failed to initialize Barcode2 equipped entity \"" << GetId() << "\".", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CBarcode2EquippedEntity::Reset() {
      for(CBarcode2Entity::TList::iterator itBarcode = m_tBarcodes.begin();
          itBarcode != m_tBarcodes.end();
          ++itBarcode) {
         (*itBarcode)->Reset();
      }
   }

   /****************************************/
   /****************************************/

   void CBarcode2EquippedEntity::Update() {
      /* Set Barcode2 position wrt reference */
      CVector3 cBarcodePosition;
      CQuaternion cBarcodeOrientation;
      for(UInt32 i = 0; i < m_tBarcodes.size(); ++i) {
         cBarcodePosition = m_vecPositionOffsets[i];
         cBarcodePosition.Rotate(m_vecPositionalEntities[i]->GetOrientation());
         cBarcodePosition += m_vecPositionalEntities[i]->GetPosition();

         cBarcodeOrientation = m_vecPositionalEntities[i]->GetOrientation() * m_vecOrientationOffsets[i];

         m_tBarcodes[i]->SetPosition(cBarcodePosition);
         m_tBarcodes[i]->SetOrientation(cBarcodeOrientation);
      }
   }
   
   /****************************************/
   /****************************************/

   CBarcode2Entity& CBarcode2EquippedEntity::GetBarcode(UInt32 un_index) {
      ARGOS_ASSERT(un_index < m_tBarcodes.size(),
                   "CBarcode2EquippedEntity::GetBarcode(), id=\"" <<
                   GetContext() + GetId() <<
                   "\": index out of bounds: un_index = " <<
                   un_index <<
                   ", m_tBarcodes.size() = " <<
                   m_tBarcodes.size());
      return *m_tBarcodes[un_index];
   }

   /****************************************/
   /****************************************/

   const CQuaternion& CBarcode2EquippedEntity::GetOffsetOrientation(UInt32 un_index) const {
      ARGOS_ASSERT(un_index < m_vecOrientationOffsets.size(),
                   "CBarcode2EquippedEntity::GetOffsetOrientation(), id=\"" <<
                   GetContext() + GetId() <<
                   "\": index out of bounds: un_index = " <<
                   un_index <<
                   ", m_vecOrientationOffsets.size() = " <<
                   m_vecOrientationOffsets.size());
      return m_vecOrientationOffsets[un_index];
   }

   /****************************************/
   /****************************************/
   
   const CVector3& CBarcode2EquippedEntity::GetOffsetPosition(UInt32 un_index) const {
      ARGOS_ASSERT(un_index < m_vecPositionOffsets.size(),
                   "CBarcode2EquippedEntity::GetOffsetPosition(), id=\"" <<
                   GetContext() + GetId() <<
                   "\": index out of bounds: un_index = " <<
                   un_index <<
                   ", m_vecPositionOffsets.size() = " <<
                   m_vecPositionOffsets.size());
      return m_vecPositionOffsets[un_index];
   }

   /****************************************/
   /****************************************/
   
   const CPositionalEntity& CBarcode2EquippedEntity::GetPositionalEntity(UInt32 un_index) const {
      ARGOS_ASSERT(un_index < m_vecPositionalEntities.size(),
                   "CBarcode2EquippedEntity::GetPositionalEntity(), id=\"" <<
                   GetContext() + GetId() <<
                   "\": index out of bounds: un_index = " <<
                   un_index <<
                   ", m_vecPositionalEntities.size() = " <<
                   m_vecPositionalEntities.size());
      return *m_vecPositionalEntities[un_index];
   }

   /****************************************/
   /****************************************/

   void CBarcode2EquippedEntity::AddToMedium(CBarcode2Medium& c_medium) {
      for(UInt32 i = 0; i < m_tBarcodes.size(); ++i) {
         m_tBarcodes[i]->AddToMedium(c_medium);
      }
   }

   /****************************************/
   /****************************************/

   void CBarcode2EquippedEntity::RemoveFromMedium(CBarcode2Medium& c_medium) {
      for(UInt32 i = 0; i < m_tBarcodes.size(); ++i) {
         m_tBarcodes[i]->RemoveFromMedium(c_medium);
      }
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_SPACE_OPERATIONS_ON_COMPOSABLE(CBarcode2EquippedEntity);

   /****************************************/
   /****************************************/

}
