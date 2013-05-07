/**
 * @file <argos3/plugins/robots/robot/simulator/frame_equipped_entity.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef FRAME_EQUIPPED_ENTITY_H
#define FRAME_EQUIPPED_ENTITY_H

namespace argos {
   class CFrameEquippedEntity;
   class CFrameEntity;
}

#include <argos3/plugins/robots/robot/simulator/frame_entity.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <map>

namespace argos {

   class CFrameEquippedEntity : public CComposableEntity {

   public:

      ENABLE_VTABLE();

      typedef std::map<std::string, CFrameEquippedEntity*> TMap;

   public:

      CFrameEquippedEntity(CComposableEntity* pc_parent);

      CFrameEquippedEntity(CComposableEntity* pc_parent,
                          const std::string& str_id);

      virtual void Init(TConfigurationNode& t_tree);
      virtual void Reset();
      virtual void Update() {}

      void AddFrame(CBodyEntty* pc_body,
                    const CVector3& c_position,
                    const CQuaternion& c_orientation);

      CFrameEntity& GetFrame(UInt32 un_index);

      inline CFrameEntity::TList& GetAllFrames() {
         return m_tFrames;
      }

      void SetFramePosition(UInt32 un_index,
                           const CVector3& c_position);

      void SetFrameOrientation(UInt32 un_index,
                              const CQuaternion& c_orientation);

      virtual std::string GetTypeDescription() const {
         return "frames";
      }

   protected:

      virtual void UpdateComponents();

   protected:

      CFrameEntity::TList m_tFrames;
   };

}

#endif
