/**
 * @file <argos3/plugins/robots/prototype/simulator/electromagnet_equipped_entity.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef ELECTROMAGNET_EQUIPPED_ENTITY_H
#define ELECTROMAGNET_EQUIPPED_ENTITY_H

namespace argos {
   class CMangetEquippedEntity;
   class CElectromagnetEntity;
}

#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/plugins/robots/prototype/simulator/electromagnet_entity.h>
#include <argos3/plugins/robots/prototype/simulator/body_entity.h>
#include <map>

namespace argos {

   /**
    * A container of CElectromagnetEntity.
    * <p>
    * This is a convenience class that acts a container of CElectromagnetEntity objects. It
    * is mostly useful when a robot is equipped with a number of electromagnets, and you
    * want to manage them comfortably.
    * </p>
    * @see CElectromagnetEntity
    */
   class CElectromagnetEquippedEntity : public CComposableEntity {

   public:

      ENABLE_VTABLE();

      typedef std::map<std::string, CElectromagnetEquippedEntity*> TMap;

   public:

      /**
       * Class constructor.
       * @param pc_parent The parent entity.
       */
      CElectromagnetEquippedEntity(CComposableEntity* pc_parent);

      /**
       * Class constructor.
       * @param pc_parent The parent entity.
       * @param str_id The id of this entity.
       */
      CElectromagnetEquippedEntity(CComposableEntity* pc_parent,
                            const std::string& str_id);

      virtual void Init(TConfigurationNode& t_tree);

      virtual void Reset();

      virtual void Update() { UpdateComponents(); }

      /**
       * Returns an electromagnet by numeric index.
       * @param un_index The index of the wanted electromagnet.
       * @return A electromagnet by numeric index.
       * @see GetAllElectromagnets()
       */
      CElectromagnetEntity& GetElectromagnet(UInt32 un_index);

      /**
       * Returns all the Electromagnets.
       * @return All the Electromagnets.
       * @see GetElectromagnet()
       */
      inline CElectromagnetEntity::TList& GetAllElectromagnets() {
         return m_tElectromagnets;
      }

      /**
       * Gets the body associated with a electromagnet.
       * @param un_index The index of the relative electromagnet.
       * @returns A reference to the body entity for the relevant electromagnet
       */
      CBodyEntity& GetElectromagnetBody(UInt32 un_index);

      /**
       * Returns the vector of bodies associated with the vector of electromagnets.
       * @returns The vector of bodies associated with the vector of electromagnets
       */
      CBodyEntity::TList& GetAllElectromagneticBodies() {
         return m_tElectromagneticBodies;
      }

      /**
       * Sets the field of an electromagnet.
       * @param un_index The index of the wanted electromagnet.
       * @param c_field The field of the electromagnet.
       */
      void SetElectromagnetField(UInt32 un_index,
                          const CVector3& c_field);

      /**
       * Sets the field of all the electromagnets to the same value.
       * @param c_field The field of the electromagnets.
       * @see SetAllElectromagnetFields()
       */
      void SetAllElectromagnetFields(const CVector3& c_field);

      virtual std::string GetTypeDescription() const {
         return "electromagnets";
      }

   protected:

      virtual void UpdateComponents() {}

   protected:

      /** A list of the Electromagnets contained in this entity */
      CElectromagnetEntity::TList m_tElectromagnets;

      /** Links to the body entities that form the electromagnets **/
      CBodyEntity::TList m_tElectromagneticBodies;

   };

}

#endif
