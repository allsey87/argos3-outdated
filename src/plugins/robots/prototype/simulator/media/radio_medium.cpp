/**
 * @file <argos3/plugins/robots/prototype/simulator/media/radio_medium.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "radio_medium.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/space/positional_indices/grid.h>
#include <argos3/core/utility/configuration/argos_exception.h>
#include <argos3/core/utility/logging/argos_log.h>

namespace argos {

   class CRxOperation : public CPositionalIndex<CRadioEntity>::COperation { 

   public:

      CRxOperation(const CRadioEntity& c_tx_radio) :
         m_cTxRadio(c_tx_radio) {}

      virtual bool operator()(CRadioEntity& c_rx_radio) {
         if((c_rx_radio.GetPosition() - m_cTxRadio.GetPosition()).Length() <  m_cTxRadio.GetTxRange()) {
            c_rx_radio.AppendRxData(m_cTxRadio.GetTxData());
         }
         return true;
      }

   private:

      const CRadioEntity& m_cTxRadio;
   };

   /****************************************/
   /****************************************/

   class CTxOperation : public CPositionalIndex<CRadioEntity>::COperation {

   public:

      CTxOperation(CPositionalIndex<CRadioEntity>& c_index) :
         m_cIndex(c_index) {}

      virtual ~CTxOperation() {
      }

      virtual bool operator()(CRadioEntity& c_tx_radio) {
         if(c_tx_radio.HasTxData()) {
            CVector3 cTxRange(c_tx_radio.GetTxRange(), c_tx_radio.GetTxRange(), c_tx_radio.GetTxRange());
            CRxOperation cRxOperation(c_tx_radio);
            m_cIndex.ForEntitiesInBoxRange(c_tx_radio.GetPosition(), cTxRange * 0.5f, cRxOperation);
         }
         return true;
      }
      
   private:

      CPositionalIndex<CRadioEntity>& m_cIndex;
   };

   /****************************************/
   /****************************************/

   CRadioMedium::CRadioMedium() {
   }

   /****************************************/
   /****************************************/

   CRadioMedium::~CRadioMedium() {
   }

   /****************************************/
   /****************************************/

   void CRadioMedium::Init(TConfigurationNode& t_tree) {
      try {
         CMedium::Init(t_tree);
         /* Get the positional index method */
         std::string strPosIndexMethod("grid");
         GetNodeAttributeOrDefault(t_tree, "index", strPosIndexMethod, strPosIndexMethod);
         /* Get the arena center and size */
         CVector3 cArenaCenter;
         CVector3 cArenaSize;
         TConfigurationNode& tArena = GetNode(CSimulator::GetInstance().GetConfigurationRoot(), "arena");
         GetNodeAttribute(tArena, "size", cArenaSize);
         GetNodeAttributeOrDefault(tArena, "center", cArenaCenter, cArenaCenter);
         /* Create the positional index for Radio entities */
         if(strPosIndexMethod == "grid") {
            size_t punGridSize[3];
            if(!NodeAttributeExists(t_tree, "grid_size")) {
               punGridSize[0] = cArenaSize.GetX();
               punGridSize[1] = cArenaSize.GetY();
               punGridSize[2] = cArenaSize.GetZ();
            }
            else {
               std::string strPosGridSize;
               GetNodeAttribute(t_tree, "grid_size", strPosGridSize);
               ParseValues<size_t>(strPosGridSize, 3, punGridSize, ',');
            }
            CGrid<CRadioEntity>* pcGrid = new CGrid<CRadioEntity>(
               cArenaCenter - cArenaSize * 0.5f, cArenaCenter + cArenaSize * 0.5f,
               punGridSize[0], punGridSize[1], punGridSize[2]);
            m_pcRadioEntityGridUpdateOperation = new CRadioEntityGridUpdater(*pcGrid);
            pcGrid->SetUpdateEntityOperation(m_pcRadioEntityGridUpdateOperation);
            m_pcRadioEntityIndex = pcGrid;
         }
         else {
            THROW_ARGOSEXCEPTION("Unknown method \"" << strPosIndexMethod << "\" for the positional index.");
         }
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Error in initialization of the Radio medium", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CRadioMedium::PostSpaceInit() {
      Update();
   }

   /****************************************/
   /****************************************/

   void CRadioMedium::Reset() {
      m_pcRadioEntityIndex->Reset();
   }

   /****************************************/
   /****************************************/

   void CRadioMedium::Destroy() {
      delete m_pcRadioEntityIndex;
      if(m_pcRadioEntityGridUpdateOperation != NULL) {
         delete m_pcRadioEntityGridUpdateOperation;
      }
   }

   /****************************************/
   /****************************************/

   void CRadioMedium::Update() {
      /* Update the positional indices of the radios */
      m_pcRadioEntityIndex->Update();
      /* Create the TX operation */
      CTxOperation cTxOperation(*m_pcRadioEntityIndex);
      /* Allow radios to communicate */
      m_pcRadioEntityIndex->ForAllEntities(cTxOperation);
   }

   /****************************************/
   /****************************************/

   void CRadioMedium::AddEntity(CRadioEntity& c_entity) {
      m_pcRadioEntityIndex->AddEntity(c_entity);
   }

   /****************************************/
   /****************************************/

   void CRadioMedium::RemoveEntity(CRadioEntity& c_entity) {
      m_pcRadioEntityIndex->RemoveEntity(c_entity);
   }

   /****************************************/
   /****************************************/

   REGISTER_MEDIUM(CRadioMedium,
                   "radio",
                   "Michael Allwright [allsey87@gmail.com]",
                   "1.0",
                   "Manages and allows the CRadioEntities to communication.",
                   "This medium is required to manage and to allow the radios to communicate\n"
                   "REQUIRED XML CONFIGURATION\n\n"
                   "<radio id=\"barcodes\" />\n\n"
                   "OPTIONAL XML CONFIGURATION\n\n"
                   "None for the time being\n",
                   "Under development"
      );

   /****************************************/
   /****************************************/

}
