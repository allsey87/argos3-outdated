/**
 * @file <argos3/plugins/robots/robotic-arm/simulator/roboticarm_entity.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef ROBOTICARM_ENTITY_H
#define ROBOTICARM_ENTITY_H

namespace argos {
   class CControllableEntity;
   class CEmbodiedEntity;
   class CRoboticArmEntity;
}

#include <argos3/core/simulator/entity/composable_entity.h>

namespace argos {

   const Real MOUNTING_POINT_RADIUS = 0.010;

   class CRoboticArmEntity : public CComposableEntity {

   public:

      ENABLE_VTABLE();

   public:

      CRoboticArmEntity();

      virtual void Init(TConfigurationNode& t_tree);
      virtual void Reset();
      virtual void Destroy();

      virtual void UpdateComponents();
      
      inline CEmbodiedEntity& GetEmbodiedEntity() {
         return *m_pcEmbodiedEntity;
      }

      //inline CControllableEntity& GetControllableEntity() {
      //   return *m_pcControllableEntity;
      //}

      virtual std::string GetTypeDescription() const {
         return "roboticarm";
      }

      inline bool IsAttachedToSomething() const {
         return m_pcAttachee != NULL;
      }

      CEmbodiedEntity& GetAttachee();

   private:

      //CControllableEntity*            m_pcControllableEntity;
      CEmbodiedEntity* m_pcEmbodiedEntity;
      CEmbodiedEntity* m_pcAttachee;
   };

}

#endif
