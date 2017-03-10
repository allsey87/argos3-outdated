
/**
 * @file <argos3/testing/srocs_loop_functions/srocs_loop_functions.cpp>
 *
 * @author Michael Allwright <allsey87@gmail.com>
 */
#include "srocs_simple_loop_functions.h"

#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/entity/embodied_entity.h>

#include <argos3/plugins/robots/prototype/simulator/entities/radio_equipped_entity.h>
#include <argos3/plugins/robots/prototype/simulator/entities/body_equipped_entity.h>

#include <iomanip>

/****************************************/
/****************************************/

void CSRoCSLoopFunctions::Init(TConfigurationNode& t_tree) {
   LOG << "[INFO] Initializing SRoCS Simple Loop Functions" << std::endl;
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

   /* uncomment for quantitative experiment */
   /*
   const CVector3 cSmallStructureCenter(0.3,-0.3,0.0);
   const CVector3 cLargeStructureCenter(-0.1,-0.1,0.0);
   std::vector<CVector3> vecBlockPositions = {
      // small structure
      cSmallStructureCenter + CVector3(0.0,0.0,0.0),
      cSmallStructureCenter + CVector3(0.0,0.0,0.055),
      // large structure
      cLargeStructureCenter + CVector3(0.0,0.0,0.0),
      cLargeStructureCenter + CVector3(0.0,0.0,0.055),
      cLargeStructureCenter + CVector3(0.0,0.0,0.110),
      cLargeStructureCenter + CVector3(0.0,0.055,0.0),
      cLargeStructureCenter + CVector3(0.0,0.055,0.055),
      cLargeStructureCenter + CVector3(0.0,-0.055,0.0),
      // unused block
      CVector3(0.25,-0.1,0.0),
   };
   */

   /* uncomment for qualitative experiment */
   const CVector3 cStructureCenter(0.0,-0.3,0.0);
   const CQuaternion cStructureRotation(-CRadians::PI_OVER_SIX, CVector3::Z);
   std::vector<CVector3> vecBlockPositions = {
      // small structure
      cStructureCenter + CVector3(0.055,0.055,0.000).Rotate(cStructureRotation),
      cStructureCenter + CVector3(0.000,0.055,0.000).Rotate(cStructureRotation),
      cStructureCenter + CVector3(0.000,0.000,0.000).Rotate(cStructureRotation),
      cStructureCenter + CVector3(0.055,0.000,0.000).Rotate(cStructureRotation),
      //cStructureCenter + CVector3(0.055,0.055,0.055),
      cStructureCenter + CVector3(0.000,0.055,0.055).Rotate(cStructureRotation),
      cStructureCenter + CVector3(0.000,0.000,0.055).Rotate(cStructureRotation),
      cStructureCenter + CVector3(0.055,0.000,0.055).Rotate(cStructureRotation),

      // unused block
      CVector3(0.35,0,0.0),
   };

   /* create blocks and insert them into the simulation */
   for(const CVector3& c_block_position : vecBlockPositions) {
      std::ostringstream strmBlockId;
      strmBlockId << c_block_position;
      CPrototypeEntity& cBlock = CreateEntity<CPrototypeEntity>("block", "block(" + strmBlockId.str() + ")", c_block_position, cStructureRotation);
      m_mapBlocks.emplace(cBlock.GetId(), &cBlock);
      AddEntity(cBlock);
   }

   /* Insert a fake radio message to initialize the target block */
   std::ostringstream strmTgtBlockId;
   strmTgtBlockId << vecBlockPositions[0];

   CRadioEquippedEntity& cBlockRadios = m_mapBlocks["block(" + strmTgtBlockId.str() + ")"]->GetComponent<CRadioEquippedEntity>("radios");
   std::vector<CByteArray> cData;
   cData.emplace_back();
   cData.back() << "1"; // Q1, target block
   cBlockRadios.GetRadio(0).SetRxData(cData);

   /* create the robots */
   m_mapRobots.emplace("robot0", &CreateEntity<CPrototypeEntity>("robot", "robot0", CVector3(0,0.1,0), CQuaternion(CRadians::ZERO, CVector3::Z)));
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

REGISTER_LOOP_FUNCTIONS(CSRoCSLoopFunctions, "srocs_simple_loop_functions");
