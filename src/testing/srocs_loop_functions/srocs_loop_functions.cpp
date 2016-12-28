
/**
 * @file <argos3/testing/srocs_loop_functions/srocs_loop_functions.cpp>
 *
 * @author Michael Allwright <allsey87@gmail.com>
 */
#include "srocs_loop_functions.h"

#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/entity/embodied_entity.h>

#include <iomanip>

/****************************************/
/****************************************/

void CSRoCSLoopFunctions::Init(TConfigurationNode& t_tree) {
   LOG << "[INFO] Initializing SRoCS Loop Functions" << std::endl;

   TConfigurationNodeIterator itEntity("entity");
   for(itEntity = itEntity.begin(&t_tree);
       itEntity != itEntity.end();
       ++itEntity) {
      std::string strEntityLabel;
      GetNodeAttribute(*itEntity, "label", strEntityLabel);
      m_mapEntityDefinitions.emplace(strEntityLabel, *itEntity);
   }
}

/****************************************/
/****************************************/

void CSRoCSLoopFunctions::Reset() {
   while(!m_mapAddedBlocks.empty()) {
      std::map<std::string, CPrototypeEntity*>::iterator itEntry = std::begin(m_mapAddedBlocks);
      RemoveEntity(*itEntry->second);
      m_mapAddedBlocks.erase(itEntry);
   }
}

/****************************************/
/****************************************/

void CSRoCSLoopFunctions::Destroy() {
   while(!m_mapAddedBlocks.empty()) {
      std::map<std::string, CPrototypeEntity*>::iterator itEntry = std::begin(m_mapAddedBlocks);
      RemoveEntity(*itEntry->second);
      m_mapAddedBlocks.erase(itEntry);
   }
   m_mapEntityDefinitions.clear();
}

/****************************************/
/****************************************/

void CSRoCSLoopFunctions::PreStep() {
   for(std::pair<const std::string, CPrototypeEntity*>& c_entry : m_mapAddedBlocks) {
      const CVector3& c_block_position = c_entry.second->GetEmbodiedEntity().GetPosition();
      if(Distance(c_block_position, m_cCacheLocation) < 0.35) {
         return;
      }
   }
   UInt32 unId = 0;
   for(;;) {
      std::ostringstream stmBlockId;
      stmBlockId << "block" << std::setfill('0') << std::setw(3) << unId;
      if(m_mapAddedBlocks.count(stmBlockId.str()) == 0) {
         CPrototypeEntity& cBlock = CreateEntity<CPrototypeEntity>("prototype", stmBlockId.str(), m_cCacheLocation, CQuaternion());
         AddEntity(cBlock);
         m_mapAddedBlocks.emplace(stmBlockId.str(), &cBlock);
         return;
      }
      unId++;
   }
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CSRoCSLoopFunctions, "srocs_loop_functions");
