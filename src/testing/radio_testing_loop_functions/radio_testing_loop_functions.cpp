
/**
 * @file <argos3/testing/srocs_loop_functions/radio_testing_functions.cpp>
 *
 * @author Michael Allwright <allsey87@gmail.com>
 */
#include "radio_testing_loop_functions.h"

#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/entity/embodied_entity.h>

#include <argos3/plugins/robots/prototype/simulator/entities/radio_equipped_entity.h>
#include <argos3/plugins/robots/prototype/simulator/entities/body_equipped_entity.h>

#include <iomanip>

/****************************************/
/****************************************/

void CRadioTestingLoopFunctions::Init(TConfigurationNode& t_tree) {
   LOG << "[INFO] Initializing Radio Testing Loop Functions" << std::endl;
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

void CRadioTestingLoopFunctions::Reset() {
   while(!m_mapRadioPads.empty()) {
      std::map<std::string, CPrototypeEntity*>::iterator itEntry = std::begin(m_mapRadioPads);
      RemoveEntity(*itEntry->second);
      m_mapRadioPads.erase(itEntry);
   }

   std::vector<CVector3> vecRadioPadPositions = {
      CVector3(0.15,0,0),
      CVector3(0.00,0,0),
      CVector3(-0.15,0,0),
      CVector3(-0.30,0,0),
   };

   /* create blocks and insert them into the simulation */
   for(const CVector3& c_radio_pad_position : vecRadioPadPositions) {
      std::ostringstream strmRadioPadId;
      strmRadioPadId << c_radio_pad_position;
      CPrototypeEntity& cRadioPad = CreateEntity<CPrototypeEntity>("radiopad", "radiopad(" + strmRadioPadId.str() + ")", c_radio_pad_position, CQuaternion());
      m_mapRadioPads.emplace(cRadioPad.GetId(), &cRadioPad);
      AddEntity(cRadioPad);
   }

   /* Insert a fake radio message to initialize the target radiopad */
   std::ostringstream strmTgtRadioPadId;
   strmTgtRadioPadId << vecRadioPadPositions[0];

   CRadioEquippedEntity& cBlockRadios = m_mapRadioPads["radiopad(" + strmTgtRadioPadId.str() + ")"]->GetComponent<CRadioEquippedEntity>("radios");
   std::vector<CByteArray> cData;
   cData.emplace_back();
   cData.back() << "1";
   cBlockRadios.GetRadio(0).SetRxData(cData);
}

/****************************************/
/****************************************/

void CRadioTestingLoopFunctions::Destroy() {
   while(!m_mapRadioPads.empty()) {
      std::map<std::string, CPrototypeEntity*>::iterator itEntry = std::begin(m_mapRadioPads);
      RemoveEntity(*itEntry->second);
      m_mapRadioPads.erase(itEntry);
   }
   m_mapEntityDefinitions.clear();
}

/****************************************/
/****************************************/

CEntity& CRadioTestingLoopFunctions::CreateEntity(const std::string& str_base_id,
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

REGISTER_LOOP_FUNCTIONS(CRadioTestingLoopFunctions, "radio_testing_loop_functions");
