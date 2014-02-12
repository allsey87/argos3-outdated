/**
 * @file <argos3/plugins/robots/prototype/simulator/radio_equipped_entity.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef RADIO_EQUIPPED_ENTITY_H
#define RADIO_EQUIPPED_ENTITY_H

namespace argos {
   class CRadioEquippedEntity;
   class CRadioEntity;
}

#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/plugins/robots/prototype/simulator/radio_entity.h>
#include <map>

namespace argos {

   /**
    * A container of CRadioEntity.
    * <p>
    * This is a container class that manages CRadioEntity objects. It
    * is required to add radios to a robot.
    * </p>
    * @see CRadioEntity
    */
   class CRadioEquippedEntity : public CComposableEntity {

   public:

      ENABLE_VTABLE();

      typedef std::map<std::string, CRadioEquippedEntity*> TMap;

   public:

      /**
       * Class constructor.
       * @param pc_parent The parent entity.
       */
      CRadioEquippedEntity(CComposableEntity* pc_parent);

      /**
       * Class constructor.
       * @param pc_parent The parent entity.
       * @param str_id The id of this entity.
       */
      CRadioEquippedEntity(CComposableEntity* pc_parent,
                           const std::string& str_id);

      virtual void Init(TConfigurationNode& t_tree);

      virtual void Reset();

      virtual void Update();


      /**
       * Returns all the radios.
       * @return All the radios.
       * @see GetRadio()
       */
      inline CRadioEntity::TList& GetAllRadios() {
         return m_tRadios;
      }

      /**
       * Returns a radio by numeric index.
       * @param un_index The index of the wanted radio.
       * @return A radio by numeric index.
       * @see GetAllRadios()
       */
      CRadioEntity& GetRadio(UInt32 un_index);

      /**
       * Returns the offset orientation of the given radio.
       * @return The offset orientation of the given radio.
       */
      const CQuaternion& GetOffsetOrientation(UInt32 un_idx) const;

      /**
       * Returns the offset position of the given radio.
       * @return The offset position of the given radio.
       */
      const CVector3& GetOffsetPosition(UInt32 un_idx) const;

      /**
       * Returns the positional entity that the given radio is attached to.
       * @return The positional entity that the given radio is attached to.
       */
      const CPositionalEntity& GetPositionalEntity(UInt32 un_idx) const;

      /**
       * Adds the radios to the target radio medium.
       * @param c_medium The target radio medium.
       * @see CRadioMedium
       */
      void AddToMedium(CRadioMedium& c_medium);

      /**
       * Removes the radios from the target radio medium.
       * @param c_medium The target radio medium.
       * @see CRadioMedium
       */
      void RemoveFromMedium(CRadioMedium& c_medium);

      virtual std::string GetTypeDescription() const {
         return "radio_container";
      }

   protected:

      /** A list of the radios contained in this entity */
      CRadioEntity::TList m_tRadios;

      /* Links to the positional entity that the 2D radios are attached to */
      std::vector<CPositionalEntity*> m_vecPositionalEntities;
      std::vector<CVector3> m_vecPositionOffsets;
      std::vector<CQuaternion> m_vecOrientationOffsets;

   };
}

#endif
