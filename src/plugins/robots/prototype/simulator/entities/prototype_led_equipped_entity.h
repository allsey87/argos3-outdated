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

      virtual void Update() { UpdateComponents(); }

      /**
       * Adds an LED to this entity.
       * @param c_position The position of the LED wrt the reference entity.
       * @param c_color The color of the LED.
       * @see GetReferenceEntity()
       */
      void AddLED(const CVector3& c_position,
                  const CColor& c_color = CColor::BLACK);

      /**
       * Adds a ring of LEDs to this entity.
       * @param c_center The position of LED ring center wrt the reference entity.
       * @param f_radius The radius of the LED ring.
       * @param c_start_angle The angle at which the first LED must be placed. Expressed wrt the local entity <em>x</em>-axis.
       * @param un_num_leds The number of LEDs to place along the ring.
       * @param c_color The color of the LEDs.
       * @see GetReferenceEntity()
       */
      void AddLEDRing(const CVector3& c_center,
                      Real f_radius,
                      const CRadians& c_start_angle,
                      UInt32 un_num_leds,
                      const CColor& c_color = CColor::BLACK);

      /**
       * Returns an LED by numeric index.
       * @param un_index The index of the wanted LED.
       * @return An LED by numeric index.
       * @see GetAllLEDs()
       */
      CLEDEntity& GetLED(UInt32 un_index);

      /**
       * Returns all the LEDs.
       * @return All the LEDs.
       * @see GetLED()
       */
      inline CLEDEntity::TList& GetAllLEDs() {
         return m_tLEDs;
      }

      /**
       * Returns the offset position of the given LED.
       * The actual position of an LED is calculated as the sum of the
       * offset position and the position of the reference.
       * @return The offset position of the given LED.
       */
      inline const CVector3& GetLEDOffsetPosition(size_t un_idx) const {
         ARGOS_ASSERT(un_idx < m_vecLEDOffsets.size(),
                      "CPrototypeLEDEquippedEntity::GetLEDOffsetPosition() : index " <<
                      un_idx <<
                      " out of bounds [0:" <<
                      m_vecLEDOffsets.size()-1 <<
                      "]" );
         return m_vecLEDOffsets[un_idx];
      }

      /**
       * Sets the position of an LED.
       * @param un_index The index of the wanted LED.
       * @param c_position The position of the LED wrt the reference entity.
       */
      void SetLEDPosition(UInt32 un_index,
                          const CVector3& c_position);

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

      virtual void UpdateComponents();

   protected:

      /** A list of the LEDs contained in this entity */
      CLEDEntity::TList m_tLEDs;

      /** Links to the positional entities that LEDs and attached to **/
      std::vector<CPositionalEntity*> m_vecLEDPositionalEntities;

      /** The offsets of the LEDs */
      std::vector<CVector3> m_vecLEDOffsets;
   };

}

#endif
