
/**
 * @file <argos3/core/simulator/entity/composable_entity.cpp>
 *
 * @author Carlo Pinciroli <ilpincy@gmail.com>
 */
#include "composable_entity.h"

#include <argos3/core/utility/string_utilities.h>

namespace argos {

   /****************************************/
   /****************************************/

   CComposableEntity::CComposableEntity(CComposableEntity* pc_parent) :
      CEntity(pc_parent) {}

   /****************************************/
   /****************************************/
   
   CComposableEntity::CComposableEntity(CComposableEntity* pc_parent,
                                        const std::string& str_id) :
      CEntity(pc_parent, str_id) {}

   /****************************************/
   /****************************************/

   void CComposableEntity::Reset() {
      for(CEntity::TMultiMap::iterator it = m_mapComponents.begin();
          it != m_mapComponents.end();
          ++it) {
         it->second->Reset();
      }
   }

   /****************************************/
   /****************************************/

   void CComposableEntity::Update() {
      UpdateComponents();
   }

   /****************************************/
   /****************************************/

   void CComposableEntity::SetEnabled(bool b_enabled) {
      CEntity::SetEnabled(b_enabled);
      for(CEntity::TMap::iterator it = m_mapComponents.begin();
          it != m_mapComponents.end();
          ++it) {
         it->second->SetEnabled(b_enabled);
      }
   }

   /****************************************/
   /****************************************/

   void CComposableEntity::UpdateComponents() {
      for(CEntity::TMap::iterator it = m_mapComponents.begin();
          it != m_mapComponents.end();
          ++it) {
         if(it->second->IsEnabled()) {
            it->second->Update();
         }
      }
   }

   /****************************************/
   /****************************************/

   void CComposableEntity::AddComponent(CEntity& c_component) {
      m_mapComponents.insert(
         std::pair<std::string, CEntity*>(
            c_component.GetTypeDescription(),
            &c_component));
   }

   /****************************************/
   /****************************************/

   CEntity& CComposableEntity::RemoveComponent(const std::string& str_component) {
      try {
         CEntity::TMultiMap::iterator it = FindComponent(str_component);
         CEntity& cRetVal = *(it->second);
         m_mapComponents.erase(it);
         return cRetVal;
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("While removing a component from a composable entity", ex);
      }
   }

   /****************************************/
   /****************************************/

   CEntity& CComposableEntity::GetComponent(const std::string& str_path) {
      try {
         /* search for the path seperator character and take the first path segement */
         size_t unFirstSeperatorIdx = str_path.find(".");
         std::string strFrontIdentifier = str_path.substr(0, unFirstSeperatorIdx);
         /* try to find the relevant component in this context */
         CEntity::TMultiMap::iterator itComponent = FindComponent(strFrontIdentifier);
         if(itComponent != m_mapComponents.end()) {
            /* path seperator not found, found component in the current context is the one we want */
            if(unFirstSeperatorIdx == std::string::npos) {
               return *(itComponent->second);
            }
            /* path seperator found, try to cast the found component to a composable entity */
            else {
               CComposableEntity* pcComposableEntity = dynamic_cast<CComposableEntity*>(itComponent->second);
               if(pcComposableEntity != NULL) {
                  /* Dynamic cast of component to composable entity was sucessful, re-execute this function in the new context */
                  return pcComposableEntity->GetComponent(str_path.substr(unFirstSeperatorIdx + 1, std::string::npos));
               }
               else {
                  /* Dynamic cast failed, user is requesting an entity from an entity which is not composable -> error */
                  THROW_ARGOSEXCEPTION("Component \"" << strFrontIdentifier << "\" of \"" << GetContext() +  GetId()
                                       << "\" is not a composable entity");
               }
            }
         }
         else {
            THROW_ARGOSEXCEPTION("Component \"" << strFrontIdentifier << "\" does not exist in \""
                                 << GetContext() +  GetId() << "\"");
         }
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("While getting a component from a composable entity", ex);
      }
   }

   /****************************************/
   /****************************************/

   /*   bool CComposableEntity::HasComponent(const std::string& str_component) {
      return m_mapComponents.count(str_component) > 0;
      }*/

   bool CComposableEntity::HasComponent(const std::string& str_path) {
      /* search for the path seperator character and take the first path segement */
      size_t unFirstSeperatorIdx = str_path.find(".");
      std::string strFrontIdentifier = str_path.substr(0, unFirstSeperatorIdx);
      /* try to find the relevant component in this context */
      CEntity::TMultiMap::iterator itComponent = FindComponent(strFrontIdentifier);
      if(itComponent != m_mapComponents.end()) {
         if(unFirstSeperatorIdx == std::string::npos) {
            /* path seperator not found, found component in the current context is the one we want */
            return true;
         }
         else {
            /* path seperator found, try to cast the found component to a composable entity */
            CComposableEntity* pcComposableEntity = dynamic_cast<CComposableEntity*>(itComponent->second);
            if(pcComposableEntity != NULL) {
               /* Dynamic cast of component to composable entity was sucessful, re-execute this function in the new context */
               return pcComposableEntity->HasComponent(str_path.substr(unFirstSeperatorIdx + 1, std::string::npos));
            }
            else {
               /* could not cast to a composable entity, the queried component cannot exist in the specified context */
               return false;
            }
         }
      }
      else {
         /* could not find the queried component in this context */
         return false;
      }
   }

   /****************************************/
   /****************************************/

   CEntity::TMultiMap::iterator CComposableEntity::FindComponent(const std::string& str_component) {            
      /* Check for the presence of [ */
      std::string::size_type unIdentifierStart = str_component.find('[');
      if(unIdentifierStart != std::string::npos) {
         /* Found, now check for the presence of ] after [ */
         std::string::size_type unIdentifierEnd = str_component.rfind(']');
         if(unIdentifierEnd != std::string::npos &&
            unIdentifierEnd > unIdentifierStart) {
            /* Use the string between [ and ] as an index and whatever comes before as base id */
            /* Count how many components there are for the base type */
            std::string strBaseType = str_component.substr(0,unIdentifierStart);
            size_t unCount = m_mapComponents.count(strBaseType);
            if(unCount == 0) {
               /* No components of this base type, return an iterator to the end of the collection */
               return m_mapComponents.end();
            }
            else {
               /* Components of base type found - extract the uid and search for it */
               std::string strComponentId = str_component.substr(unIdentifierStart + 1, unIdentifierEnd - unIdentifierStart - 1);
               /* create an pair of iterators which mark the beginning and the end of the components that match the base type */
               std::pair<CEntity::TMultiMap::iterator,
                         CEntity::TMultiMap::iterator> cRange = m_mapComponents.equal_range(strBaseType);
               /* create an iterator to hold the component we are trying to locate */
               CEntity::TMultiMap::iterator itComponent;
               /* search through components of base type and try find a match for the specified Id */
               for(itComponent = cRange.first;
                   itComponent != cRange.second;
                   ++itComponent) {
                  if(itComponent->second->GetId() == strComponentId) {
                     break;
                  }
               }
               /* if the iterator itComponent is not equal to cRange.second, then we have found our component */
               if(itComponent != cRange.second) {
                  return itComponent;
               }
               else {
                  /* Identifer not found in the collection of components with the specified base type,
                   * return an iterator to the end of the collect to show this */
                  return m_mapComponents.end();
               }
            }
         }
         else {
            THROW_ARGOSEXCEPTION("Syntax error in entity id \"" << str_component << "\"");
         }
      }
      else {
         /* Identifier syntax not used, return an iterator to the first element or the end of collection if
          * no elements are found */
         return m_mapComponents.find(str_component);
      }
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_SPACE_OPERATIONS_ON_COMPOSABLE(CComposableEntity);

   /****************************************/
   /****************************************/

}
