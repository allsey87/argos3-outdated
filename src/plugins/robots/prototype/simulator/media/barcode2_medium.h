/**
 * @file <argos3/plugins/robots/prototype/simulator/media/barcode2_medium.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef BARCODE2_MEDIUM_H
#define BARCODE2_MEDIUM_H

namespace argos {
   class CBarcode2Medium;
   class CBarcode2Entity;
}

#include <argos3/core/simulator/medium/medium.h>
#include <argos3/core/simulator/space/positional_indices/positional_index.h>
#include <argos3/plugins/robots/prototype/simulator/entities/barcode2_entity.h>

namespace argos {

   class CBarcode2Medium : public CMedium {

   public:

      /**
       * Class constructor.
       */
      CBarcode2Medium();

      /**
       * Class destructor.
       */
      virtual ~CBarcode2Medium();

      virtual void Init(TConfigurationNode& t_tree);
      virtual void PostSpaceInit();
      virtual void Reset();
      virtual void Destroy();
      virtual void Update();

     /**
      * Adds the specified entity to the list of managed entities.
      * @param c_entity The entity to add.
      */
      void AddEntity(CBarcode2Entity& c_entity);

     /**
      * Removes the specified entity from the list of managed entities.
      * @param c_entity The entity to remove.
      */
      void RemoveEntity(CBarcode2Entity& c_entity);

      /**
       * Returns the barcode2 positional index.
       * @return The barcode2 positional index.
       */
      CPositionalIndex<CBarcode2Entity>& GetIndex() {
         return *m_pcBarcode2EntityIndex;
      }

   private:

      /** A positional index for the barcode2 entities */
      CPositionalIndex<CBarcode2Entity>* m_pcBarcode2EntityIndex;

      /** The update operation for the grid positional index */
      CBarcode2EntityGridUpdater* m_pcBarcode2EntityGridUpdateOperation;

   };

}

#endif
