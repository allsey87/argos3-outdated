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

   template<class S, class... ARG_TS>
   std::shared_ptr<S> AddState(const std::string& str_id, ARG_TS&&... args) {
      return std::shared_ptr<S>(new S(str_id, this, std::forward<ARG_TS>(args)...));
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

   template <class S>
   S& GetBase() {
      if(HasParent()) {
         GetParent().GetBase<S>();
      }
      else {
         S* pcCasted = dynamic_cast<S*>(this);
         if(pcCasted != nullptr) {
            return *pcCasted;
         }
         else {
            std::logic_error(m_strId + " can not be converted to requested type");
         }
      }
   }
   
   friend std::ostream& operator<<(std::ostream& c_stream, const CState& c_state);

private:
   /* definition of a transition */
   struct STransition {
      std::vector<std::shared_ptr<CState> >::iterator FromState;
      std::vector<std::shared_ptr<CState> >::iterator ToState;
      std::function<bool()> Guard;
   };

private:
   /* state name */
   std::string m_strId;
   /* parent */
   CState* m_pcParent;
   /* entry and exit methods */
   std::function<void()> m_fnEntryMethod;
   /* substates */
   std::vector<std::shared_ptr<CState> > m_vecStates;
   /* iterator to the current substate */
   std::vector<std::shared_ptr<CState> >::iterator m_itCurrentState;
   /* collection of transitions */
   std::vector<STransition> m_vecTransitions;
};

std::ostream& operator<<(std::ostream& c_stream, const CState& c_state);

#endif

