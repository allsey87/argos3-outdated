/**
 * @file <argos3/plugins/robots/prototype/simulator/prototype_entity.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "prototype_entity.h"

#include <argos3/plugins/robots/prototype/simulator/body_equipped_entity.h>
#include <argos3/plugins/robots/prototype/simulator/joint_equipped_entity.h>

#include <argos3/plugins/robots/prototype/simulator/prototype_proximity_sensor_equipped_entity.h>
#include <argos3/plugins/robots/prototype/simulator/prototype_led_equipped_entity.h>
#include <argos3/plugins/robots/prototype/simulator/forwards_camera_equipped_entity.h>
#include <argos3/plugins/robots/prototype/simulator/electromagnet_equipped_entity.h>
#include <argos3/plugins/robots/prototype/simulator/barcode2_equipped_entity.h>

#include <argos3/core/simulator/space/space.h>
//#include <argos3/core/simulator/entity/controllable_entity.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/simulator.h>

namespace argos {

   /****************************************/
   /****************************************/

   CPrototypeEntity::CPrototypeEntity() :
      CComposableEntity(NULL),
      m_pcControllableEntity(NULL),
      m_pcEmbodiedEntity(NULL),
      m_pcBodyEquippedEntity(NULL),
      m_pcJointEquippedEntity(NULL) {}

   /****************************************/
   /****************************************/

   void CPrototypeEntity::Init(TConfigurationNode& t_tree) {
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
         m_pcEmbodiedEntity->Init(GetNode(t_tree, "body"));

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

         if(NodeExists(t_tree, "devices")) {
            TConfigurationNodeIterator itDevice;
            for(itDevice = itDevice.begin(&GetNode(t_tree, "devices"));
                itDevice != itDevice.end();
                ++itDevice) {

               if(itDevice->Value() == "proximity_sensors") {
                  CPrototypeProximitySensorEquippedEntity* m_pcEquippedEntity = 
                     new CPrototypeProximitySensorEquippedEntity(this);
                  m_pcEquippedEntity->Init(*itDevice);
                  AddComponent(*m_pcEquippedEntity);
               }
               else if(itDevice->Value() == "leds" ) {
                  CPrototypeLEDEquippedEntity* m_pcEquippedEntity =
                     new CPrototypeLEDEquippedEntity(this);
                  m_pcEquippedEntity->Init(*itDevice);
                  AddComponent(*m_pcEquippedEntity);
               }
               else if(itDevice->Value() == "forwards_cameras" ) {
                  CForwardsCameraEquippedEntity* m_pcEquippedEntity = 
                     new CForwardsCameraEquippedEntity(this);
                  m_pcEquippedEntity->Init(*itDevice);
                  AddComponent(*m_pcEquippedEntity);         
               }
               else if(itDevice->Value() == "electromagnets" ) {
                  CElectromagnetEquippedEntity* m_pcEquippedEntity = 
                     new CElectromagnetEquippedEntity(this);
                  m_pcEquippedEntity->Init(*itDevice);
                  AddComponent(*m_pcEquippedEntity);         
               }
               else if(itDevice->Value() == "barcodes" ) {
                  CBarcode2EquippedEntity* m_pcEquippedEntity = 
                     new CBarcode2EquippedEntity(this);
                  m_pcEquippedEntity->Init(*itDevice);
                  AddComponent(*m_pcEquippedEntity);         
               }
               else {
THROW_ARGOSEXCEPTION("Attempt to add unimplemented device type \"" << itDevice->Value() << "\".");
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

   void CPrototypeEntity::Reset() {
      /* Reset all components */
      CComposableEntity::Reset();
      /* Update components */
      UpdateComponents();
   }

   /****************************************/
   /****************************************/

   void CPrototypeEntity::Destroy() {
      CComposableEntity::Destroy();
   }

   /****************************************/
   /****************************************/

   void CPrototypeEntity::UpdateComponents() {
      CComposableEntity::UpdateComponents();
   }


   /****************************************/
   /****************************************/

   REGISTER_ENTITY(CPrototypeEntity,
                   "prototype",
                   "1.0",
                   "Michael Allwright [allsey87@gmail.com]",
                   "A generic physics engine driven, configurable implementation of a prototype",
                   "[long description]",
                   "Under development"
   );

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_SPACE_OPERATIONS_ON_COMPOSABLE(CPrototypeEntity);

   /****************************************/
   /****************************************/
}
