/**
 * @file <argos3/plugins/robots/prototype/simulator/barcode2_equipped_entity.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "barcode2_medium.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/space/positional_indices/grid.h>
#include <argos3/core/utility/configuration/argos_exception.h>
#include <argos3/core/utility/logging/argos_log.h>

namespace argos {

   /****************************************/
   /****************************************/

   CBarcode2Medium::CBarcode2Medium() {
   }

   /****************************************/
   /****************************************/

   CBarcode2Medium::~CBarcode2Medium() {
   }

   /****************************************/
   /****************************************/

   void CBarcode2Medium::Init(TConfigurationNode& t_tree) {
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
         /* Create the positional index for Barcode2 entities */
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
            CGrid<CBarcode2Entity>* pcGrid = new CGrid<CBarcode2Entity>(
               cArenaCenter - cArenaSize * 0.5f, cArenaCenter + cArenaSize * 0.5f,
               punGridSize[0], punGridSize[1], punGridSize[2]);
            m_pcBarcode2EntityGridUpdateOperation = new CBarcode2EntityGridUpdater(*pcGrid);
            pcGrid->SetUpdateEntityOperation(m_pcBarcode2EntityGridUpdateOperation);
            m_pcBarcode2EntityIndex = pcGrid;
         }
         else {
            THROW_ARGOSEXCEPTION("Unknown method \"" << strPosIndexMethod << "\" for the positional index.");
         }
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Error in initialization of the Barcode2 medium", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CBarcode2Medium::PostSpaceInit() {
      Update();
   }

   /****************************************/
   /****************************************/

   void CBarcode2Medium::Reset() {
      m_pcBarcode2EntityIndex->Reset();
   }

   /****************************************/
   /****************************************/

   void CBarcode2Medium::Destroy() {
      delete m_pcBarcode2EntityIndex;
      if(m_pcBarcode2EntityGridUpdateOperation != NULL) {
         delete m_pcBarcode2EntityGridUpdateOperation;
      }
   }

   /****************************************/
   /****************************************/

   void CBarcode2Medium::Update() {
      m_pcBarcode2EntityIndex->Update();
   }

   /****************************************/
   /****************************************/

   void CBarcode2Medium::AddEntity(CBarcode2Entity& c_entity) {
      m_pcBarcode2EntityIndex->AddEntity(c_entity);
   }

   /****************************************/
   /****************************************/

   void CBarcode2Medium::RemoveEntity(CBarcode2Entity& c_entity) {
      m_pcBarcode2EntityIndex->RemoveEntity(c_entity);
   }

   /****************************************/
   /****************************************/

   REGISTER_MEDIUM(CBarcode2Medium,
                   "barcode2",
                   "Michael Allwright [allsey87@gmail.com]",
                   "1.0",
                   "Manages the 2D Barcodes.",
                   "This medium is required to manage the 2D barcode entities, thus allowing the\n"
                   "associated camera sensors to work properly. If you intend to use a camera\n"
                   "sensor that detects barcodes, you must add this medium to the XML\n"
                   "configuration file.\n\n"
                   "REQUIRED XML CONFIGURATION\n\n"
                   "<barcode2 id=\"barcodes\" />\n\n"
                   "OPTIONAL XML CONFIGURATION\n\n"
                   "None for the time being\n",
                   "Under development"
      );

   /****************************************/
   /****************************************/

}
