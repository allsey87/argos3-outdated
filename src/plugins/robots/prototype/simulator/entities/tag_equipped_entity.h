/**
 * @file <argos3/plugins/robots/prototype/simulator/entities/tag_equipped_entity.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef TAG_EQUIPPED_ENTITY_H
#define TAG_EQUIPPED_ENTITY_H

namespace argos {
   class CTagEquippedEntity;
   class CTagEntity;
}

#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/plugins/robots/prototype/simulator/entities/tag_entity.h>
#include <map>

namespace argos {

   /**
    * A container of CTagEntity.
    * <p>
    * This is a convenience class that acts a container of CTagEntity objects. It
    * is mostly useful when an entity is equipped with a number of tags, and you
    * want to manage them comfortably.
    * </p>
    * @see CTagEntity
    */
   class CTagEquippedEntity : public CComposableEntity {

   public:

      ENABLE_VTABLE();

      typedef std::map<std::string, CTagEquippedEntity*> TMap;

   public:

      /**
       * Class constructor.
       * @param pc_parent The parent entity.
       */
      CTagEquippedEntity(CComposableEntity* pc_parent);

      /**
       * Class constructor.
       * @param pc_parent The parent entity.
       * @param str_id The id of this entity.
       */
      CTagEquippedEntity(CComposableEntity* pc_parent,
                              const std::string& str_id);

      virtual void Init(TConfigurationNode& t_tree);

      virtual void Reset();

      virtual void Update();


      /**
       * Returns all the tags.
       * @return All the tags.
       * @see GetTag()
       */
      inline CTagEntity::TList& GetAllTags() {
         return m_tTags;
      }

      /**
       * Returns a tag by numeric index.
       * @param un_index The index of the wanted 2D tag.
       * @return A tag by numeric index.
       * @see GetAllLEDs()
       */
      CTagEntity& GetTag(UInt32 un_index);

      /**
       * Returns the offset orientation of the given tag.
       * @return The offset orientation of the given tag.
       */
      const CQuaternion& GetOffsetOrientation(UInt32 un_idx) const;

      /**
       * Returns the offset position of the given tag.
       * @return The offset position of the given tag.
       */
      const CVector3& GetOffsetPosition(UInt32 un_idx) const;

      /**
       * Returns the positional entity that the given tag is attached to.
       * @return The positional entity that the given tag is attached to.
       */
      const CPositionalEntity& GetPositionalEntity(UInt32 un_idx) const;

      /**
       * Adds the tags to the target tag medium.
       * @param c_medium The target tag medium.
       * @see CTagMedium
       */
      void AddToMedium(CTagMedium& c_medium);

      /**
       * Removes the tags from the target tag medium.
       * @param c_medium The target tag medium.
       * @see CTagMedium
       */
      void RemoveFromMedium(CTagMedium& c_medium);

      virtual std::string GetTypeDescription() const {
         return "tag_container";
      }

   protected:

      /** A list of the tags contained in this entity */
      CTagEntity::TList m_tTags;

      /* Links to the positional entity that the tags are attached to */
      std::vector<CPositionalEntity*> m_vecPositionalEntities;
      std::vector<CVector3> m_vecPositionOffsets;
      std::vector<CQuaternion> m_vecOrientationOffsets;

   };

}

#endif
