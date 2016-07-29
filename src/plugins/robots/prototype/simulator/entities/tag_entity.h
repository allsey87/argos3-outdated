/**
 * @file <argos3/plugins/robots/prototype/simulator/entities/tag_entity.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef TAG_ENTITY_H
#define TAG_ENTITY_H

namespace argos {
   class CTagEntity;
   class CTagMedium;
}

#include <argos3/core/simulator/entity/positional_entity.h>
#include <argos3/core/utility/datatypes/set.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/datatypes/byte_array.h>
#include <argos3/core/simulator/space/positional_indices/space_hash.h>
#include <argos3/core/simulator/space/positional_indices/grid.h>

namespace argos {

   class CTagEntity : public CPositionalEntity {

   public:

      ENABLE_VTABLE();

      typedef std::vector<CTagEntity*> TList;
      typedef CSet<CTagEntity*> TSet;

   public:

      CTagEntity(CComposableEntity* pc_parent);

      CTagEntity(CComposableEntity* pc_parent,
                      const std::string& str_id,
                      const std::string& str_payload,
                      bool b_localisable,
                      Real f_side_length);
                                            
      virtual ~CTagEntity() {}

      virtual void Init(TConfigurationNode& t_tree);

      virtual void Reset() {}

      virtual void SetEnabled(bool b_enabled) {}

      /**
       * Returns the payload of the tag.
       * @return the payload of the tag.
       * @see SetPayload()
       */
      inline const std::string& GetPayload() const {
         return m_strPayload;
      }

      /**
       * Sets the payload of the tag to the provided data.
       * @param str_payload the new data.
       * @see GetPayload()
       */
      inline void SetPayload(const std::string& str_payload) {
         m_strPayload = str_payload;
      }

      inline Real GetSideLength() const {
         return m_fSideLength;
      }
      
      inline bool IsLocalizable() const {
         return m_bLocalizable;
      }


      virtual std::string GetTypeDescription() const {
         return "tag";
      }

      /**
       * Adds the tag to the desired medium.
       * @param c_medium The medium.
       * @see CTagMedium
       */
      void AddToMedium(CTagMedium& c_medium);

      /**
       * Removes the tag from the desired medium.
       * @param c_medium The medium.
       * @see CTagMedium
       */
      void RemoveFromMedium(CTagMedium& c_medium);

   protected:

      std::string m_strPayload;
      bool m_bLocalizable;
      Real m_fSideLength;
   };

   /****************************************/
   /****************************************/

   class CTagEntitySpaceHashUpdater : public CSpaceHashUpdater<CTagEntity> {

   public:

      virtual void operator()(CAbstractSpaceHash<CTagEntity>& c_space_hash,
                              CTagEntity& c_element);

   private:

      SInt32 m_nI, m_nJ, m_nK;

   };

   /****************************************/
   /****************************************/

   class CTagEntityGridUpdater : public CGrid<CTagEntity>::COperation {

   public:

      CTagEntityGridUpdater(CGrid<CTagEntity>& c_grid);
      virtual bool operator()(CTagEntity& c_entity);

   private:

      CGrid<CTagEntity>& m_cGrid;
      SInt32 m_nI, m_nJ, m_nK;

   };

   /****************************************/
   /****************************************/

}

#endif
