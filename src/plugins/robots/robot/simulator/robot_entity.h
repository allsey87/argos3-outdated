/**
 * @file <argos3/plugins/robots/robot/simulator/robot_entity.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef ROBOT_ENTITY_H
#define ROBOT_ENTITY_H

namespace argos {
   class CControllableEntity;
   class CEmbodiedEntity;
   class CJointEquippedEntity;
   class CRobotEntity;
}

#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/plugins/robots/robot/simulator/body_equipped_entity.h>
#include <argos3/plugins/robots/robot/simulator/joint_equipped_entity.h>

namespace argos {

   class CRobotEntity : public CComposableEntity {

   public:

      ENABLE_VTABLE();

   public:

      CRobotEntity();

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

      //inline CControllableEntity& GetControllableEntity() {
      //   return *m_pcControllableEntity;
      //}

      virtual std::string GetTypeDescription() const {
         return "robot";
      }

   private:

      //CControllableEntity*            m_pcControllableEntity;
      CEmbodiedEntity* m_pcEmbodiedEntity;
      CBodyEquippedEntity* m_pcBodyEquippedEntity;
      CJointEquippedEntity* m_pcJointEquippedEntity;
   };

}

#endif
