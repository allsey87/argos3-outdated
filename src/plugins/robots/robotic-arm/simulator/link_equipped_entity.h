/**
 * @file <argos3/plugins/robots/robotic-arm/simulator/link_equipped_entity.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef LINK_EQUIPPED_ENTITY_H
#define LINK_EQUIPPED_ENTITY_H

namespace argos {
   class CLinkEquippedEntity;
   class CLinkEntity;
}

#include <argos3/plugins/robots/robotic-arm/simulator/link_entity.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <map>

namespace argos {

   class CLinkEquippedEntity : public CComposableEntity {

   public:

      ENABLE_VTABLE();

      typedef std::map<std::string, CLinkEquippedEntity*> TMap;

   public:

      CLinkEquippedEntity(CComposableEntity* pc_parent);

      CLinkEquippedEntity(CComposableEntity* pc_parent,
                          const std::string& str_id);

      virtual void Init(TConfigurationNode& t_tree);
      virtual void Reset();
      virtual void Update() {}

      void AddLink(const CVector3& c_position,
                   const CQuaternion& c_orientation);

      CLinkEntity& GetLink(UInt32 un_index);

      inline CLinkEntity::TList& GetAllLinks() {
         return m_tLinks;
      }

      void SetLinkPosition(UInt32 un_index,
                           const CVector3& c_position);

      void SetLinkOrientation(UInt32 un_index,
                              const CQuaternion& c_orientation);

      virtual std::string GetTypeDescription() const {
         return "links";
      }

   protected:

      virtual void UpdateComponents();

   protected:

      CLinkEntity::TList m_tLinks;
   };

}

#endif
