
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

   CSRoCSLoopFunctions() {}

   void Init(TConfigurationNode& t_tree);

   void Reset();

   void Destroy();

   void PreStep() {}

   virtual ~CSRoCSLoopFunctions() {}

private:

   template <class E>
   E& CreateEntity(const std::string& str_base_id, /* robot, block etc */
                   const std::string& str_id,
                   const CVector3& c_position,
                   const CQuaternion& c_quaternion) {
      CEntity* pcEntity = &CreateEntity(str_base_id, str_id, c_position, c_quaternion);
      E* pcEntityCasted = dynamic_cast<E*>(pcEntity);
      if(pcEntityCasted != NULL) {
         return *pcEntityCasted;
      }
      else {
         THROW_ARGOSEXCEPTION("Type conversion failed for entity \"" << str_id << "\"");
      }
   }

   CEntity& CreateEntity(const std::string& str_base_id,
                         const std::string& str_id,
                         const CVector3& c_position,
                         const CQuaternion& c_quaternion);

private:

   std::map<std::string, TConfigurationNode> m_mapEntityDefinitions;
   std::map<std::string, CPrototypeEntity*> m_mapBlocks;
   std::map<std::string, CPrototypeEntity*> m_mapRobots;
};

#endif
