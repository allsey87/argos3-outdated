/**
 * @file <argos3/plugins/robots/robot/simulator/robot_entity.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "robot_entity.h"

#include <argos3/plugins/robots/robot/simulator/body_equipped_entity.h>
#include <argos3/plugins/robots/robot/simulator/joint_equipped_entity.h>

#include <argos3/plugins/simulator/entities/proximity_sensor_equipped_entity.h>

#include <argos3/core/simulator/space/space.h>
//#include <argos3/core/simulator/entity/controllable_entity.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/simulator.h>

namespace argos {

   /****************************************/
   /****************************************/

   CRobotEntity::CRobotEntity() :
      CComposableEntity(NULL),
      m_pcControllableEntity(NULL),
      m_pcEmbodiedEntity(NULL),
      m_pcBodyEquippedEntity(NULL),
      m_pcJointEquippedEntity(NULL) {}

   /****************************************/
   /****************************************/

   void CRobotEntity::Init(TConfigurationNode& t_tree) {
      try {
         /*
          * Init parent
          */
         CComposableEntity::Init(t_tree);
         
         /*
          * Create and init components
          */
         /* Embodied entity */
         m_pcEmbodiedEntity = new CEmbodiedEntity(this);
         AddComponent(*m_pcEmbodiedEntity);
         m_pcEmbodiedEntity->Init(t_tree);

         m_pcBodyEquippedEntity = new CBodyEquippedEntity(this);
         AddComponent(*m_pcBodyEquippedEntity);
         if(NodeExists(t_tree, "bodies")) {
            m_pcBodyEquippedEntity->Init(GetNode(t_tree, "bodies"));
         }

         m_pcJointEquippedEntity = new CJointEquippedEntity(this);
         AddComponent(*m_pcJointEquippedEntity);
         if(NodeExists(t_tree, "joints")) {
            m_pcJointEquippedEntity->Init(GetNode(t_tree, "joints"));
         }

         if(NodeExists(t_tree, "sensors")) {
            TConfigurationNodeIterator itSensor;
            for(itSensor = itSensor.begin(&GetNode(t_tree, "sensors"));
                itSensor != itSensor.end();
                ++itSensor) {
               
               std::string strTargetBody;
               GetNodeAttribute(*itSensor, "body", strTargetBody);
               CBodyEntity& cSensorBody = GetComponent<CBodyEntity>("bodies.body[" + strTargetBody + "]");

               if(itSensor->Value() == "proximity") {
                  CProximitySensorEquippedEntity* m_pcSensorEquippedEntity = 
                     new CProximitySensorEquippedEntity(&cSensorBody);
                  m_pcSensorEquippedEntity->Init(*itSensor);
                  cSensorBody.AddComponent(*m_pcSensorEquippedEntity);
               }
               else {
THROW_ARGOSEXCEPTION("Attempt to add unimplemented sensor type \"" << itSensor->Value() << "\".");
               }
            }
         }

         /* Controllable entity
            It must be the last one, for actuators/sensors to link to composing entities correctly */
         if(NodeExists(t_tree, "controller")) {
            m_pcControllableEntity = new CControllableEntity(this);
            AddComponent(*m_pcControllableEntity);
            m_pcControllableEntity->Init(GetNode(t_tree, "controller"));
         }
         /* Update components */
         UpdateComponents();
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Failed to initialize entity \"" << GetId() << "\".", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CRobotEntity::Reset() {
      /* Reset all components */
      CComposableEntity::Reset();
      /* Update components */
      UpdateComponents();
   }

   /****************************************/
   /****************************************/

   void CRobotEntity::Destroy() {
      CComposableEntity::Destroy();
   }

   /****************************************/
   /****************************************/

   void CRobotEntity::UpdateComponents() {
      m_pcEmbodiedEntity->Update();
   }


   /****************************************/
   /****************************************/

   REGISTER_ENTITY(CRobotEntity,
                   "robot",
                   "1.0",
                   "Michael Allwright [allsey87@gmail.com]",
                   "A generic physics engine driven, configurable implementation of a robot",
                   "[long description]",
                   "Under development"
   );

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_SPACE_OPERATIONS_ON_COMPOSABLE(CRobotEntity);

   /****************************************/
   /****************************************/
}
