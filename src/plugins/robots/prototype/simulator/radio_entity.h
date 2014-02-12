/**
 * @file <argos3/plugins/robots/prototype/simulator/radio_entity.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef RADIO_ENTITY_H
#define RADIO_ENTITY_H

namespace argos {
   class CRadioEntity;
   class CRadioMedium;
}

#include <argos3/core/simulator/entity/positional_entity.h>
#include <argos3/core/utility/datatypes/set.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/datatypes/byte_array.h>
#include <argos3/core/simulator/space/positional_indices/space_hash.h>
#include <argos3/core/simulator/space/positional_indices/grid.h>

namespace argos {

   class CRadioEntity : public CPositionalEntity {

   public:

      ENABLE_VTABLE();

   public:

      typedef std::vector<CRadioEntity*> TList;

   public:

      CRadioEntity(CComposableEntity* pc_parent);

      CRadioEntity(CComposableEntity* pc_parent,
                   const std::string& str_id,
                   Real f_transmit_range);
                                            
      virtual ~CRadioEntity() {}

      virtual void Init(TConfigurationNode& t_tree);

      virtual void Reset() {}

      virtual void SetEnabled(bool b_enabled) {}

      /**
       * Returns the received data of the radio.
       * @return the received data of the radio.
       * @see SetRxData()
       * @see AppendRxData()
       */
      inline const std::vector<CByteArray>& GetRxData() const {
         return m_vecRxData;
      }

      /**
       * Returns the received data of the radio.
       * @return the received data of the radio.
       * @see SetRxData()
       * @see AppendRxData()
       */
      inline std::vector<CByteArray>& GetRxData() {
         return m_vecRxData;
      }

      /**
       * Sets the data received by the radio
       * @param vec_data the new data.
       * @see GetRxData()
       */
      inline void SetRxData(const std::vector<CByteArray>& vec_data) {
         m_vecRxData = vec_data;
      }

      /**
       * Appends the data recieved by the radio
       * @param vec_data the new data.
       * @see GetRxData()
       */
      inline void AppendRxData(const std::vector<CByteArray>& vec_data) {
         m_vecRxData.insert(m_vecRxData.end(),
                            vec_data.begin(),
                            vec_data.end());
      }

      /**
       * Clears the data received by the radio
       * @see HasRxData()
       */
      inline void ClearRxData() {
         m_vecRxData.clear();
      }

      /**
       * Checks if there has been data received by the radio
       * @return A boolean value representing whether data has been received
       * @see ClearRxData()
       */
      inline bool HasRxData() const {
         return !m_vecRxData.empty();
      }

      /**
       * Returns the data waiting to be transmitted by the radio
       * @return The data waiting to be transmitted by the radio
       * @see SetTxData()
       * @see AppendTxData()
       */
      inline const std::vector<CByteArray>& GetTxData() const {
         return m_vecTxData;
      }

      /**
       * Returns the data waiting to be transmitted by the radio
       * @return The data waiting to be transmitted by the radio
       * @see SetTxData()
       * @see AppendTxData()
       */
      inline std::vector<CByteArray>& GetTxData() {
         return m_vecTxData;
      }

      /**
       * Sets the data waiting to be transmitted by the radio
       * @param vec_data the new data
       * @see GetTxData()
       */
      inline void SetTxData(const std::vector<CByteArray>& vec_data) {
         m_vecTxData = vec_data;
      }

      /**
       * Appends data to be transmitted by the radio.
       * @param vec_data the new data.
       * @see GetTxData()
       */
      inline void AppendTxData(const std::vector<CByteArray>& vec_data) {
         m_vecTxData.insert(m_vecTxData.end(),
                            vec_data.begin(),
                            vec_data.end());
      }

      /**
       * Clears the data waiting to be transmitted by the radio.
       * @see HasTxData()
       */
      inline void ClearTxData() {
         m_vecTxData.clear();
      }

     /**
       * Checks if there is data waiting to be transmitted by the radio.
       * @return A boolean value representing whether there is data for transmitting
       * @see GetTxData()
       */ 
     inline bool HasTxData() {
         return !m_vecTxData.empty();
      }

      /**
       * Returns the transmission range of the radio.
       * @return the transmission range of the radio.
       * @see SetTxRange()
       */
      inline Real GetTxRange() const {
         return m_fTxRange;
      }

      /**
       * Sets the tranmission range of the radio
       * @param f_range the new transmission range of the radio in metres.
       * @see GetTxRange()
       */
      inline void SetTxRange(Real f_range) {
         m_fTxRange = f_range;
      }


      virtual std::string GetTypeDescription() const {
         return "radio";
      }

      /**
       * Adds the radio to the desired medium.
       * @param c_medium The medium.
       * @see CradioMedium
       */
      void AddToMedium(CRadioMedium& c_medium);

      /**
       * Removes the radio from the desired medium.
       * @param c_medium The medium.
       * @see CradioMedium
       */
      void RemoveFromMedium(CRadioMedium& c_medium);

   protected:

      Real m_fTxRange;

      std::vector<CByteArray> m_vecRxData;
      std::vector<CByteArray> m_vecTxData;

   };

   /****************************************/
   /****************************************/

   class CRadioEntitySpaceHashUpdater : public CSpaceHashUpdater<CRadioEntity> {

   public:

      virtual void operator()(CAbstractSpaceHash<CRadioEntity>& c_space_hash,
                              CRadioEntity& c_element);

   private:

      SInt32 m_nI, m_nJ, m_nK;

   };

   /****************************************/
   /****************************************/

   class CRadioEntityGridUpdater : public CGrid<CRadioEntity>::COperation {

   public:

      CRadioEntityGridUpdater(CGrid<CRadioEntity>& c_grid);
      virtual bool operator()(CRadioEntity& c_entity);

   private:

      CGrid<CRadioEntity>& m_cGrid;
      SInt32 m_nI, m_nJ, m_nK;

   };

   /****************************************/
   /****************************************/

}

#endif
