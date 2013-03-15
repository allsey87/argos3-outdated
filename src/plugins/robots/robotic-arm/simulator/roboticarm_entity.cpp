/**
 * @file <argos3/plugins/robots/robotic-arm/simulator/roboticarm_entity.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "roboticarm_entity.h"

#include <argos3/core/utility/math/matrix/rotationmatrix3.h>
#include <argos3/core/simulator/space/space.h>
//#include <argos3/core/simulator/entity/controllable_entity.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/simulator.h>

namespace argos {

   /****************************************/
   /****************************************/

   CRoboticArmEntity::CRoboticArmEntity() :
      CComposableEntity(NULL),
      //m_pcControllableEntity(NULL),
      m_pcEmbodiedEntity(NULL),
      m_pcAttachee(NULL) {}

   /****************************************/
   /****************************************/

   void CRoboticArmEntity::Init(TConfigurationNode& t_tree) {
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
         
         /* Controllable entity
            It must be the last one, for actuators/sensors to link to composing entities correctly */
         //m_pcControllableEntity = new CControllableEntity(this);
         //AddComponent(*m_pcControllableEntity);
         //m_pcControllableEntity->Init(t_tree);
         /* Update components */
         UpdateComponents();
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Failed to initialize entity \"" << GetId() << "\".", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CRoboticArmEntity::Reset() {
      /* Reset all components */
      CComposableEntity::Reset();
      /* Update components */
      UpdateComponents();
   }

   /****************************************/
   /****************************************/

   void CRoboticArmEntity::Destroy() {
      CComposableEntity::Destroy();
   }

   /****************************************/
   /****************************************/

   void CRoboticArmEntity::UpdateComponents() {
      m_pcEmbodiedEntity->Update();
   }


   /****************************************/
   /****************************************/

   REGISTER_ENTITY(CRoboticArmEntity,
                   "robotic-arm",
                   "1.0",
                   "Michael Allwright [allsey87@gmail.com]",
                   "[short description]",
                   "[long description]",
                   "Under development"
   );

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_SPACE_OPERATIONS_ON_COMPOSABLE(CRoboticArmEntity);

   /****************************************/
   /****************************************/
}
