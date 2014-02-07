/**
 * @file <argos3/plugins/robots/prototype/simulator/barcode2_entity.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef BARCODE2_ENTITY_H
#define BARCODE2_ENTITY_H

namespace argos {
   class CBarcode2Entity;
   class CBarcode2Medium;
}

#include <argos3/core/simulator/entity/positional_entity.h>
#include <argos3/core/utility/datatypes/set.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/datatypes/byte_array.h>
#include <argos3/core/simulator/space/positional_indices/space_hash.h>
#include <argos3/core/simulator/space/positional_indices/grid.h>

namespace argos {

   class CBarcode2Entity : public CPositionalEntity {

   public:

      ENABLE_VTABLE();

      typedef std::vector<CBarcode2Entity*> TList;
      typedef CSet<CBarcode2Entity*> TSet;

   public:

      CBarcode2Entity(CComposableEntity* pc_parent);

      CBarcode2Entity(CComposableEntity* pc_parent,
                      const std::string& str_id,
                      const std::string& str_payload,
                      bool b_localisable,
                      Real f_side_length);
                                            
      virtual ~CBarcode2Entity() {}

      virtual void Init(TConfigurationNode& t_tree);

      virtual void Reset() {}

      virtual void SetEnabled(bool b_enabled) {}

      /**
       * Returns the payload of the Barcode2.
       * @return the payload of the Barcode2.
       * @see SetPayload()
       */
      inline const std::string& GetPayload() const {
         return m_strPayload;
      }

      /**
       * Sets the payload of the Barcode2 to the provided data.
       * @param str_payload the new data.
       * @see GetPayload()
       */
      inline void SetPayload(const std::string& str_payload) {
         m_strPayload = str_payload;
      }

      inline Real GetSideLength() const {
         return m_fSideLength;
      }


      virtual std::string GetTypeDescription() const {
         return "barcode2";
      }

      /**
       * Adds the 2D barcode to the desired medium.
       * @param c_medium The medium.
       * @see CBarcode2Medium
       */
      void AddToMedium(CBarcode2Medium& c_medium);

      /**
       * Removes the 2D barcode from the desired medium.
       * @param c_medium The medium.
       * @see CBarcode2Medium
       */
      void RemoveFromMedium(CBarcode2Medium& c_medium);

   protected:

      std::string m_strPayload;
      bool m_bLocalizable;
      Real m_fSideLength;
   };

   /****************************************/
   /****************************************/

   class CBarcode2EntitySpaceHashUpdater : public CSpaceHashUpdater<CBarcode2Entity> {

   public:

      virtual void operator()(CAbstractSpaceHash<CBarcode2Entity>& c_space_hash,
                              CBarcode2Entity& c_element);

   private:

      SInt32 m_nI, m_nJ, m_nK;

   };

   /****************************************/
   /****************************************/

   class CBarcode2EntityGridUpdater : public CGrid<CBarcode2Entity>::COperation {

   public:

      CBarcode2EntityGridUpdater(CGrid<CBarcode2Entity>& c_grid);
      virtual bool operator()(CBarcode2Entity& c_entity);

   private:

      CGrid<CBarcode2Entity>& m_cGrid;
      SInt32 m_nI, m_nJ, m_nK;

   };

   /****************************************/
   /****************************************/

}

#endif
