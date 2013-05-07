/**
 * @file <argos3/plugins/robots/robot/simulator/joint_entity.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef JOINT_ENTITY_H
#define JOINT_ENTITY_H

namespace argos {
   class CJointEntity;
   class CBodyEntity;
}

#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/plugins/robots/robot/simulator/body_equipped_entity.h>

namespace argos {

   class CJointEntity : public CComposableEntity {

   public:

      ENABLE_VTABLE();

      typedef std::vector<CJointEntity*> TList;

   public:
      class CFrame : public CEntity {
         CFrame(CComposableEntity* pc_parent,
                const std::string& str_id,
                CPositionEntity* pc_reference_frame,
                CBodyEntity* pc_reference_body) :
            m_pcReferenceFrame(pc_reference_frame),
            m_pcReferenceBody(pc_reference_body) {}

         CPositionalEntity* m_pcReferenceFrame;
         CBodyEntity* m_pcReferenceBody;
      };

   public:

      CJointEntity(CComposableEntity* pc_parent);

      CJointEntity(CComposableEntity* pc_parent,
                   const std::string& str_id,
                   bool b_disable_collisions,);

      virtual ~CJointEntity() {}

      virtual void Init(TConfigurationNode& t_tree);
      virtual void Reset();
      virtual void Destroy();

      virtual void UpdateComponents();

      CBodyEntity& GetConnectedBody(UInt32 un_index);
            
      CBodyEntity::TList& GetAllConnectedBodies() {
         return m_tConnectedBodies;
      }

      virtual std::string GetTypeDescription() const {
         return "joint";
      }



   private:      
      EJointType m_eJointType;

      bool m_bDisableCollisions;

      std::vector<CPositionalEntity *> m_vecJointFrames;

   public:

   };

}

#endif
