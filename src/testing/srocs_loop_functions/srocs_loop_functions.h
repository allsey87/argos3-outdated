
/**
 * @file <argos3/testing/srocs_loop_functions/srocs_loop_functions.h>
 *
 * @author Michael Allwright <allsey87@gmail.com>
 */
#ifndef SROCS_LOOP_FUNCTIONS_H
#define SROCS_LOOP_FUNCTIONS_H

#include <argos3/plugins/robots/prototype/simulator/entities/prototype_entity.h>

#include <argos3/core/simulator/loop_functions.h>

using namespace argos;

class CSRoCSLoopFunctions : public CLoopFunctions {

public:

   CSRoCSLoopFunctions() :
      m_cCacheLocation(0.35,0.0,0.125) {}


   void Init(TConfigurationNode& t_tree);

   void Reset();

   void Destroy();

   void PreStep();

   virtual ~CSRoCSLoopFunctions() {}

private:

   template <class E>
   E& CreateEntity(const std::string& str_label, 
                   const std::string& str_id,
                   const CVector3& c_position,
                   const CQuaternion& c_quaternion) {
      CEntity* pcEntity;
      try {
         /* Create entity */
         pcEntity = CFactory<CEntity>::New(str_label);
         /* Copy XML configuration */
         TConfigurationNode tEntityNode = m_mapEntityDefinitions[str_label];
         /* Set its ID */
         SetNodeAttribute(tEntityNode, "id", str_id);
         /* If the tree does not have a 'body' node, create a new one */
         if(!NodeExists(tEntityNode, "body")) {
            TConfigurationNode tBodyNode("body");
            AddChildNode(tEntityNode, tBodyNode);
         }
         /* Get 'body' node */
         TConfigurationNode& tBodyNode = GetNode(tEntityNode, "body");
         /* Set the position */
         SetNodeAttribute(tBodyNode, "position", c_position);
         /* Set the orientation */
         SetNodeAttribute(tBodyNode, "orientation", c_quaternion);
         /* Init the entity (this also creates the components, if pcEntity is a composable) */
         pcEntity->Init(tEntityNode);
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Error while trying to create entity", ex);
      }
      E* pcEntityCasted = dynamic_cast<E*>(pcEntity);
      if(pcEntityCasted != NULL) {
         return *pcEntityCasted;
      }
      else {
         THROW_ARGOSEXCEPTION("Type conversion failed for entity \"" << str_id << "\" of type \"" << str_label);
      }
   }

private:

   std::map<std::string, TConfigurationNode> m_mapEntityDefinitions;
   std::map<std::string, CPrototypeEntity*> m_mapAddedBlocks;

   CVector3 m_cCacheLocation;

};

#endif
