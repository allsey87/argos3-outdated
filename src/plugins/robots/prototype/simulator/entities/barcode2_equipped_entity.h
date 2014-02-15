/**
 * @file <argos3/plugins/robots/prototype/simulator/entities/barcode2_equipped_entity.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef BARCODE2_EQUIPPED_ENTITY_H
#define BARCODE2_EQUIPPED_ENTITY_H

namespace argos {
   class CBarcode2EquippedEntity;
   class CBarcode2Entity;
}

#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/plugins/robots/prototype/simulator/entities/barcode2_entity.h>
#include <map>

namespace argos {

   /**
    * A container of CBarcode2Entity.
    * <p>
    * This is a convenience class that acts a container of CBarcode2Entity objects. It
    * is mostly useful when a robot is equipped with a number of 2D barcodes, and you
    * want to manage them comfortably.
    * </p>
    * @see CBarcode2Entity
    */
   class CBarcode2EquippedEntity : public CComposableEntity {

   public:

      ENABLE_VTABLE();

      typedef std::map<std::string, CBarcode2EquippedEntity*> TMap;

   public:

      /**
       * Class constructor.
       * @param pc_parent The parent entity.
       */
      CBarcode2EquippedEntity(CComposableEntity* pc_parent);

      /**
       * Class constructor.
       * @param pc_parent The parent entity.
       * @param str_id The id of this entity.
       */
      CBarcode2EquippedEntity(CComposableEntity* pc_parent,
                              const std::string& str_id);

      virtual void Init(TConfigurationNode& t_tree);

      virtual void Reset();

      virtual void Update();


      /**
       * Returns all the 2D barcodes.
       * @return All the 2D barcodes.
       * @see GetBarcode2()
       */
      inline CBarcode2Entity::TList& GetAllBarcodes() {
         return m_tBarcodes;
      }

      /**
       * Returns a 2D barcode by numeric index.
       * @param un_index The index of the wanted 2D barcode.
       * @return A barcode by numeric index.
       * @see GetAllLEDs()
       */
      CBarcode2Entity& GetBarcode(UInt32 un_index);

      /**
       * Returns the offset orientation of the given barcode.
       * @return The offset orientation of the given barcode.
       */
      const CQuaternion& GetOffsetOrientation(UInt32 un_idx) const;

      /**
       * Returns the offset position of the given barcode.
       * @return The offset position of the given barcode.
       */
      const CVector3& GetOffsetPosition(UInt32 un_idx) const;

      /**
       * Returns the positional entity that the given barcode is attached to.
       * @return The positional entity that the given barcode is attached to.
       */
      const CPositionalEntity& GetPositionalEntity(UInt32 un_idx) const;

      /**
       * Adds the barcodes to the target barcode medium.
       * @param c_medium The target barcode medium.
       * @see CBarcode2Medium
       */
      void AddToMedium(CBarcode2Medium& c_medium);

      /**
       * Removes the barcodes from the target barcode medium.
       * @param c_medium The target barcode medium.
       * @see CBarcode2Medium
       */
      void RemoveFromMedium(CBarcode2Medium& c_medium);

      virtual std::string GetTypeDescription() const {
         return "barcode2_container";
      }

   protected:

      /** A list of the 2D barcodes contained in this entity */
      CBarcode2Entity::TList m_tBarcodes;

      /* Links to the positional entity that the 2D barcodes are attached to */
      std::vector<CPositionalEntity*> m_vecPositionalEntities;
      std::vector<CVector3> m_vecPositionOffsets;
      std::vector<CQuaternion> m_vecOrientationOffsets;

   };

}

#endif
