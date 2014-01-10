/**
 * @file <argos3/plugins/robots/prototype/simulator/robot_entity.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef PROTOTYPE_ENTITY_H
#define PROTOTYPE_ENTITY_H

namespace argos {
   class CControllableEntity;
   class CEmbodiedEntity;
   class CJointEquippedEntity;
   class CPrototypeEntity;
}

#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/plugins/robots/prototype/simulator/body_equipped_entity.h>
#include <argos3/plugins/robots/prototype/simulator/joint_equipped_entity.h>

namespace argos {

   class CPrototypeEntity : public CComposableEntity {

   public:

      ENABLE_VTABLE();

   public:

      CPrototypeEntity();

      virtual void Init(TConfigurationNode& t_tree);
      virtual void Reset();
      virtual void Destroy();

      virtual void UpdateComponents();
      
      inline CEmbodiedEntity& GetEmbodiedEntity() {
         return *m_pcEmbodiedEntity;
      }

      inline CBodyEquippedEntity& GetBodyEquippedEntity() {
         return *m_pcBodyEquippedEntity;
      }

      inline CJointEquippedEntity& GetJointEquippedEntity() {
         return *m_pcJointEquippedEntity;
      }

      inline bool HasControllableEntity() {
         return m_pcControllableEntity != NULL;
      }

      inline CControllableEntity& GetControllableEntity() {
         return *m_pcControllableEntity;
      }

      virtual std::string GetTypeDescription() const {
         return "prototype";
      }

   private:

      CControllableEntity*            m_pcControllableEntity;
      CEmbodiedEntity* m_pcEmbodiedEntity;
      CBodyEquippedEntity* m_pcBodyEquippedEntity;
      CJointEquippedEntity* m_pcJointEquippedEntity;
   };

}

#endif
