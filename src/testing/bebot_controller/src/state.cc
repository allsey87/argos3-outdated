#include "state.h"

#include <iostream>

CState::CState(const std::string& str_id,
               CState* pc_parent,
               std::function<void()> fn_entry_method,
               const std::vector<std::shared_ptr<CState> >& vec_states) :
   m_strId(str_id),
   m_pcParent(pc_parent),
   m_fnEntryMethod(fn_entry_method),
   m_ptrData(nullptr),
   m_vecStates(vec_states) {
   m_itCurrentState = std::end(m_vecStates);
}

/***********************************************************/
/***********************************************************/

CState& CState::GetState(const std::string& str_id) {
   std::vector<std::shared_ptr<CState> >::iterator itState = 
      std::find_if(std::begin(m_vecStates), std::end(m_vecStates), [&str_id] (const std::shared_ptr<CState>& pc_state) {
         return (pc_state->m_strId == str_id);
      });

   if(itState == std::end(m_vecStates)) {
      throw std::invalid_argument(str_id + " doesn't exist in " + m_strId);
   }
   return **itState;
}

/***********************************************************/
/***********************************************************/
     
void CState::SetEntryFunction(std::function<void()> fn_entry_method) {
   m_fnEntryMethod = fn_entry_method;
}

/***********************************************************/
/***********************************************************/

void CState::AddTransition(std::string str_from_state,
                           std::string str_to_state,
                           std::function<bool()> fn_guard) {
   std::vector<std::shared_ptr<CState> >::iterator itFromState = 
      std::find_if(std::begin(m_vecStates),
                   std::end(m_vecStates),
                   [&str_from_state] (const std::shared_ptr<CState>& pc_state) {
      return (pc_state->m_strId == str_from_state);
   });
   std::vector<std::shared_ptr<CState> >::iterator itToState = 
      std::find_if(std::begin(m_vecStates),
                   std::end(m_vecStates),
                   [&str_to_state] (const std::shared_ptr<CState>& pc_state) {
      return (pc_state->m_strId == str_to_state);
   });
   if(itFromState != std::end(m_vecStates)) {
      if(itToState != std::end(m_vecStates)) {
         m_vecTransitions.push_back( {itFromState, itToState, fn_guard} );
      }
      else {
         throw std::invalid_argument(str_to_state + " doesn't exist in " + m_strId);
      }
   }
   else {
      throw std::invalid_argument(str_from_state + " doesn't exist in " + m_strId);
   }
}

/***********************************************************/
/***********************************************************/

void CState::AddExitTransition(std::string str_from_state,
                               std::function<bool()> fn_guard) {
                   
   std::vector<std::shared_ptr<CState> >::iterator itFromState =
      std::find_if(std::begin(m_vecStates),
                   std::end(m_vecStates),
                   [&str_from_state] (const std::shared_ptr<CState>& pc_state) {
      return (pc_state->m_strId == str_from_state);
   });
   if(itFromState != std::end(m_vecStates)) {
      m_vecTransitions.push_back( {itFromState, std::end(m_vecStates), fn_guard} );
   }
   else {
      throw std::invalid_argument(str_from_state + " doesn't exist in " + m_strId);
   }
}

/***********************************************************/
/***********************************************************/

bool CState::Step() { 
   /* Run entry procedure if defined and set initial sub-state */
   if(m_itCurrentState == std::end(m_vecStates)) {
      if(m_fnEntryMethod != nullptr) {
         m_fnEntryMethod();
      }
      /* set the iterator to the first substate */
      m_itCurrentState = std::begin(m_vecStates);
   }
   else {
      bool bTransitionReq = (*m_itCurrentState)->Step();
      /* If the step routine of the substate returns true, it is done and a 
         transition to neighboring state is required */
      if(bTransitionReq) {
         for(const STransition& s_transition : m_vecTransitions) {
            if(s_transition.FromState == m_itCurrentState && s_transition.Guard()) {
               m_itCurrentState = s_transition.ToState;
               break;
            }
         }
      }
   }
   /* if the current substate points to the end of the substates, we have either
      no substates, or we are doing an exit transition */
   return (m_itCurrentState == std::end(m_vecStates));
}

/***********************************************************/
/***********************************************************/

std::ostream& operator<<(std::ostream& c_stream, const CState& c_state) {
   c_stream << c_state.m_strId;
   if(c_state.m_itCurrentState != std::end(c_state.m_vecStates)) {
      c_stream << "." << **c_state.m_itCurrentState;
   }
   return c_stream;
}

/***********************************************************/
/***********************************************************/

