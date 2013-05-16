/**
 * @file <argos3/plugins/robots/robot/simulator/joint_entity.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef JOINT_ENTITY_H
#define JOINT_ENTITY_H

namespace argos {
   class CJointEntity;
   class CFrameEquippedEntity;
}

#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/core/simulator/entity/composable_entity.h>
//#include <argos3/plugins/robots/robot/simulator/frame_equipped_entity.h>

namespace argos {

   class CJointEntity : public CComposableEntity {

   public:

      ENABLE_VTABLE();

      typedef std::vector<CJointEntity*> TList;

   public:

      CJointEntity(CComposableEntity* pc_parent);

      CJointEntity(CComposableEntity* pc_parent,
                   const std::string& str_id,
                   bool b_disable_collisions);

      virtual ~CJointEntity() {}

      virtual void Init(TConfigurationNode& t_tree);
      virtual void Reset();
      virtual void Destroy();

      virtual void UpdateComponents();

      CFrameEquippedEntity& GetFrameEquippedEntity() {
         return *m_pcFrameEquippedEntity;
      }

      virtual std::string GetTypeDescription() const {
         return "joint";
      }

      const CVector3& GetLinearLowerLimit() const {
         return m_cLinearLowerLimit;
      }

      const CVector3& GetLinearUpperLimit() const {
         return m_cLinearUpperLimit;
      }

      const CVector3& GetAngularLowerLimit() const {
         return m_cAngularLowerLimit;
      }

      const CVector3& GetAngularUpperLimit() const {
         return m_cAngularUpperLimit;
      }

      bool GetDisableLinkedBodyCollisions() const {
         return m_bDisableCollisions;
      }

   private:      
      bool m_bDisableCollisions;

      CFrameEquippedEntity* m_pcFrameEquippedEntity;

      CVector3 m_cLinearLowerLimit;
      CVector3 m_cLinearUpperLimit;
      CVector3 m_cAngularLowerLimit;
      CVector3 m_cAngularUpperLimit;
      
   public:

   };

}

#endif
