/**
 * @file <argos3/plugins/robots/prototype/simulator/entities/prototype_led_equipped_entity.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef PROTOTYPE_LED_EQUIPPED_ENTITY_H
#define PROTOTYPE_LED_EQUIPPED_ENTITY_H

namespace argos {
   class CPrototypeLEDEquippedEntity;
   class CLEDEntity;
}

#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/plugins/simulator/entities/led_entity.h>
#include <map>

namespace argos {

   /**
    * A container of CLEDEntity.
    * <p>
    * This is a convenience class that acts a container of CLEDEntity objects. It
    * is mostly useful when a robot is equipped with a number of LEDs, and you
    * want to manage them comfortably.
    * </p>
    * <p>
    * You can define a positional entity as the <em>reference</em> of this entity.
    * In this way, if the reference entity moves, this entity will follow automatically.
    * The contained LEDs will also move accordingly. If you don't define a reference
    * entity, the LEDs won't move.
    * </p>
    * @see CLEDEntity
    */
   class CPrototypeLEDEquippedEntity : public CComposableEntity {

   public:

      ENABLE_VTABLE();

      typedef std::map<std::string, CPrototypeLEDEquippedEntity*> TMap;

   public:

      /**
       * Class constructor.
       * @param pc_parent The parent entity.
       * @param pc_reference The positional entity to which this entity refers for its position.
       */
      CPrototypeLEDEquippedEntity(CComposableEntity* pc_parent);

      /**
       * Class constructor.
       * @param pc_parent The parent entity.
       * @param str_id The id of this entity.
       * @param pc_reference The positional entity to which this entity refers for its position.
       */
      CPrototypeLEDEquippedEntity(CComposableEntity* pc_parent,
                                  const std::string& str_id);

      virtual void Init(TConfigurationNode& t_tree);

      virtual void Reset();

      virtual void Update();

      /**
       * Returns all the LEDs.
       * @return All the LEDs.
       * @see GetLED()
       */
      inline CLEDEntity::TList& GetAllLEDs() {
         return m_tLEDs;
      }

      /**
       * Returns an LED by numeric index.
       * @param un_index The index of the wanted LED.
       * @return An LED by numeric index.
       * @see GetAllLEDs()
       */
      CLEDEntity& GetLED(UInt32 un_index);

      /**
       * Returns the offset orientation of the given LED.
       * @return The offset orientation of the given LED.
       */
      const CQuaternion& GetOffsetOrientation(UInt32 un_idx) const;

      /**
       * Returns the offset position of the given LED.
       * @return The offset position of the given LED.
       */
      const CVector3& GetOffsetPosition(UInt32 un_idx) const;

      /**
       * Returns the positional entity that the given LED is attached to.
       * @return The positional entity that the given LED is attached to.
       */
      const CPositionalEntity& GetPositionalEntity(UInt32 un_idx) const;

      /**
       * Sets the color of an LED.
       * @param un_index The index of the wanted LED.
       * @param c_color The color of the LED.
       */
      void SetLEDColor(UInt32 un_index,
                       const CColor& c_color);

      /**
       * Sets the color of all the LEDs to the same value.
       * @param c_color The color of the LEDs.
       * @see SetAllLEDsColors()
       */
      void SetAllLEDsColors(const CColor& c_color);

      /**
       * Sets the color of all the LEDs to the given setting.
       * @param vec_colors A vector containing the colors of the LEDs.
       * @see SetAllLEDsColors()
       * @throws CARGoSException if the size of the passed vector is different from the number of LEDs.
       */
      void SetAllLEDsColors(const std::vector<CColor>& vec_colors);

      /**
       * Adds the LEDs to the wanted LED medium.
       * @param c_medium The LED medium.
       * @see CLEDMedium
       */
      void AddToMedium(CLEDMedium& c_medium);

      /**
       * Removes the LEDs from the wanted LED medium.
       * @param c_medium The LED medium.
       * @see CLEDMedium
       */
      void RemoveFromMedium(CLEDMedium& c_medium);

      virtual std::string GetTypeDescription() const {
         return "leds";
      }

   protected:

      /** A list of the LEDs contained in this entity */
      CLEDEntity::TList m_tLEDs;

      std::vector<CPositionalEntity*> m_vecPositionalEntities;
      std::vector<CVector3> m_vecPositionOffsets;
      std::vector<CQuaternion> m_vecOrientationOffsets;

   };

}

#endif
