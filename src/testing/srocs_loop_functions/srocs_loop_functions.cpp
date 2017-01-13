
/**
 * @file <argos3/testing/srocs_loop_functions/srocs_loop_functions.cpp>
 *
 * @author Michael Allwright <allsey87@gmail.com>
 */
#include "srocs_loop_functions.h"

#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/entity/embodied_entity.h>

#include <argos3/plugins/robots/prototype/simulator/entities/radio_equipped_entity.h>

#include <iomanip>

/****************************************/
/****************************************/

void CSRoCSLoopFunctions::Init(TConfigurationNode& t_tree) {
   LOG << "[INFO] Initializing SRoCS Loop Functions" << std::endl;
   /* parse the configuration */
   TConfigurationNodeIterator itEntity("entity");
   for(itEntity = itEntity.begin(&t_tree);
       itEntity != itEntity.end();
       ++itEntity) {
      std::string strBaseId;
      GetNodeAttribute(*itEntity, "id", strBaseId);
      m_mapEntityDefinitions.emplace(strBaseId, *itEntity);
   }
   /* call reset to init the environment */
   Reset();
}

/****************************************/
/****************************************/

void CSRoCSLoopFunctions::Reset() {
   while(!m_mapBlocks.empty()) {
      std::map<std::string, CPrototypeEntity*>::iterator itEntry = std::begin(m_mapBlocks);
      RemoveEntity(*itEntry->second);
      m_mapBlocks.erase(itEntry);
   }
   while(!m_mapRobots.empty()) {
      std::map<std::string, CPrototypeEntity*>::iterator itEntry = std::begin(m_mapRobots);
      RemoveEntity(*itEntry->second);
      m_mapRobots.erase(itEntry);
   }
   /* create the seed block */
   CPrototypeEntity& cSeedBlock = CreateEntity<CPrototypeEntity>("block", "seed", CVector3(), CQuaternion());
   /* Insert a fake radio message to initialize the seed block */
   CRadioEquippedEntity& cBlockRadios = cSeedBlock.GetComponent<CRadioEquippedEntity>("radios");
   std::vector<CByteArray> cData;
   cData.emplace_back();
   cData.back() << "3"; // Q3, center stack for pyramid
   cBlockRadios.GetRadio(0).SetRxData(cData);
   AddEntity(cSeedBlock);
   m_mapBlocks.emplace("seed", &cSeedBlock);

   /////////////// TESTING ///////////////////
   /*
   UInt32 i = 10;
   for(const std::pair<std::string, CVector3>& c_block_config : std::vector<std::pair<std::string, CVector3> > {
      std::make_pair("3", CVector3(0,0,0.055)),
      std::make_pair("3", CVector3(0,0,0.110)),
      std::make_pair("2", CVector3(0,0.055,0)),
      std::make_pair("2", CVector3(0,-0.055,0)),
      std::make_pair("2", CVector3(0,0.055,0.055)),
      std::make_pair("2", CVector3(0,-0.055,0.055)),
      std::make_pair("1", CVector3(0,0.110,0)),
      std::make_pair("1", CVector3(0,-0.110,0)),

      //std::make_pair("2", CVector3(0.055,0,0.0)),
      //std::make_pair("2", CVector3(0.055,0,0.055)),
   }) {
      CPrototypeEntity& cBlock = CreateEntity<CPrototypeEntity>("block", "test" + std::to_string(i++), c_block_config.second, CQuaternion());
      CRadioEquippedEntity& cRadios = cBlock.GetComponent<CRadioEquippedEntity>("radios");
      std::vector<CByteArray> cRxData;
      cRxData.emplace_back();
      cRxData.back() << c_block_config.first; // Q3, center stack for pyramid
      cRadios.GetRadio(0).SetRxData(cRxData);
      AddEntity(cBlock);
   }
   */

   /////////////// TESTING ///////////////////

   /* create the robots */
   m_mapRobots.emplace("robot0", &CreateEntity<CPrototypeEntity>("robot", "robot0", CVector3(0.20,0,0), CQuaternion(CRadians::ZERO, CVector3::Z)));
   m_mapRobots.emplace("robot1", &CreateEntity<CPrototypeEntity>("robot", "robot1", CVector3(0,0.20,0), CQuaternion(CRadians::PI_OVER_TWO, CVector3::Z)));
   m_mapRobots.emplace("robot2", &CreateEntity<CPrototypeEntity>("robot", "robot2", CVector3(-0.20,0,0), CQuaternion(CRadians::PI, CVector3::Z)));
   m_mapRobots.emplace("robot3", &CreateEntity<CPrototypeEntity>("robot", "robot3", CVector3(0,-0.20,0), CQuaternion(-CRadians::PI_OVER_TWO, CVector3::Z)));
   /* add robots to simulation */
   for(const std::pair<const std::string, CPrototypeEntity*>& c_pair : m_mapRobots) {
      AddEntity(*(c_pair.second));
   }

}

/****************************************/
/****************************************/

void CSRoCSLoopFunctions::Destroy() {
   while(!m_mapBlocks.empty()) {
      std::map<std::string, CPrototypeEntity*>::iterator itEntry = std::begin(m_mapBlocks);
      RemoveEntity(*itEntry->second);
      m_mapBlocks.erase(itEntry);
   }
   while(!m_mapRobots.empty()) {
      std::map<std::string, CPrototypeEntity*>::iterator itEntry = std::begin(m_mapRobots);
      RemoveEntity(*itEntry->second);
      m_mapRobots.erase(itEntry);
   }
   m_mapEntityDefinitions.clear();
}

/****************************************/
/****************************************/

void CSRoCSLoopFunctions::PreStep() {
   for(const CVector3& c_cache_location : m_vecCacheLocations) {
      bool bBlockFound = false;
      for(std::pair<const std::string, CPrototypeEntity*>& c_entry : m_mapBlocks) {
         const CVector3& c_block_position = c_entry.second->GetEmbodiedEntity().GetPosition();
         if(Distance(c_block_position, c_cache_location) < 0.35) {
            bBlockFound = true;
            break;
         }
      }
      if(bBlockFound) {
         continue;
      }
      else {
         UInt32 unId = 0;
         for(;;) {
            std::ostringstream stmBlockId;
            stmBlockId << "block" << std::setfill('0') << std::setw(3) << unId;
            if(m_mapBlocks.count(stmBlockId.str()) == 0) {
               CPrototypeEntity& cBlock = CreateEntity<CPrototypeEntity>("block", stmBlockId.str(), c_cache_location, CQuaternion(CRadians::ZERO, CVector3::Z));
               AddEntity(cBlock);
               m_mapBlocks.emplace(stmBlockId.str(), &cBlock);
               break;
            }
            unId++;
         }
      }
   }
}

/****************************************/
/****************************************/

CEntity& CSRoCSLoopFunctions::CreateEntity(const std::string& str_base_id,
                                           const std::string& str_id,
                                           const CVector3& c_position,
                                           const CQuaternion& c_quaternion) {
   CEntity* pcEntity;
   try {
      /* Get a reference to the entity */
      TConfigurationNodeIterator itEntity;
      itEntity = itEntity.begin(&m_mapEntityDefinitions[str_base_id]);
      /* Copy XML configuration */
      TConfigurationNode tEntityNode = *itEntity;
      pcEntity = CFactory<CEntity>::New(tEntityNode.Value());
      /* Set its ID */
      SetNodeAttribute(tEntityNode, "id", str_id);
      /* If the tree does not have a 'body' node, create a new one */
      if(!NodeExists(tEntityNode, "body")) {
         TConfigurationNode tBodyNode("body");
         AddChildNode(tEntityNode, tBodyNode);
      }
      /* Set the position and orientation of the body */
      TConfigurationNode& tBodyNode = GetNode(tEntityNode, "body");
      SetNodeAttribute(tBodyNode, "position", c_position);
      SetNodeAttribute(tBodyNode, "orientation", c_quaternion);
      /* Init the entity (this also creates the components, if pcEntity is a composable) */
      pcEntity->Init(tEntityNode);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error while trying to create entity", ex);
   }
   return *pcEntity;
}

REGISTER_LOOP_FUNCTIONS(CSRoCSLoopFunctions, "srocs_loop_functions");
