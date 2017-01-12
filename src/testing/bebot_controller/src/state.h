#ifndef STATE_H
#define STATE_H

#include <string>
#include <vector>
#include <functional>
#include <memory>
#include <algorithm>
#include <ostream>

class CState {
public:
   using TVector = std::vector<std::shared_ptr<CState> >;

public:
   CState(const std::string& str_id,
          CState* pc_parent,
          std::function<void()> fn_entry_method = nullptr,
          const std::vector<std::shared_ptr<CState> >& vec_sub_states = {});

   virtual ~CState() {}

   CState& GetState(const std::string& str_id);
        
   void SetEntryFunction(std::function<void()> fn_entry_method);
     
   void AddTransition(std::string str_from_state,
                      std::string str_to_state,
                      std::function<bool()> fn_guard = [] { return true; });

   void AddExitTransition(std::string str_from_state,
                          std::function<bool()> fn_guard = [] { return true; });
      
   bool Step();

   template<class STATE, class... ARG_TS>
   std::shared_ptr<STATE> AddState(const std::string& str_id, ARG_TS&&... args) {
      return std::make_shared<STATE>(str_id, this, std::forward<ARG_TS>(args)...);
   }

   const std::string& GetId() {
      return m_strId;
   }

   bool HasParent() {
      return (m_pcParent != nullptr);
   }

   CState& GetParent() {
      if(m_pcParent != nullptr) {
         return *m_pcParent;
      }
      else {
         throw std::logic_error(m_strId + " does not have a parent");
      } 
   }

   CState& GetBase() {
      if(HasParent()) {
         return GetParent().GetBase();
      }
      else {
         return *this;
      }
   }

   template <class STATE>
   STATE& GetBase() {
      STATE* pcCasted = dynamic_cast<STATE*>(&GetBase());
      if(pcCasted != nullptr) {
         return *pcCasted;
      }
      else {
         throw std::logic_error(m_strId + " can not be converted to requested type");
      }
   }

   template<class DATA, class... ARG_TS>
   void SetData(ARG_TS&&... args) {
      m_ptrData = std::make_shared<DATA>(std::forward<ARG_TS>(args)...);
   }

   template <class DATA>
   DATA& GetData() {
      if(m_ptrData == nullptr) {
         throw std::logic_error(m_strId + " does not have data");
      }
      std::shared_ptr<DATA> ptrData = std::dynamic_pointer_cast<DATA>(m_ptrData);
      if(ptrData == nullptr) {
         throw std::logic_error("cannot convert data to the requested type");
      }
      return *ptrData;
   }
      
   friend std::ostream& operator<<(std::ostream& c_stream, const CState& c_state);

public:
   /* base class for local data */
   struct SData {
      virtual ~SData() {};
   };

private:
   /* definition of a transition */
   struct STransition {
      std::vector<std::shared_ptr<CState> >::iterator FromState;
      std::vector<std::shared_ptr<CState> >::iterator ToState;
      std::function<bool()> Guard;
   };

//TODO: revert to private
public:
   /* state name */
   std::string m_strId;
   /* parent */
   CState* m_pcParent;
   /* entry and exit methods */
   std::function<void()> m_fnEntryMethod;
   /* pointer to local data */
   std::shared_ptr<SData> m_ptrData;
   /* substates */
   std::vector<std::shared_ptr<CState> > m_vecStates;
   /* iterator to the current substate */
   std::vector<std::shared_ptr<CState> >::iterator m_itCurrentState;
   /* collection of transitions */
   std::vector<STransition> m_vecTransitions;
};

std::ostream& operator<<(std::ostream& c_stream, const CState& c_state);

#endif

