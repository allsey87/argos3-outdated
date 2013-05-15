
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
         CEntity* pcEntity = FindComponent(strFrontIdentifier)->second;
         /* path seperator not found, found component in the current context is the one we want */
         if(unFirstSeperatorIdx == std::string::npos) {
            return *pcEntity;
         }
         /* path seperator found, try to cast the found component to a composable entity */
         else {
            CComposableEntity* pcComposableEntity = dynamic_cast<CComposableEntity*>(pcEntity);
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
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("While getting a component from a composable entity", ex);
      }
   }

   /****************************************/
   /****************************************/

   bool CComposableEntity::HasComponent(const std::string& str_component) {
      return m_mapComponents.count(str_component) > 0;
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
               /* No components of this base type -> error */
               THROW_ARGOSEXCEPTION("No component of type \"" << strBaseType << "\" found for entity \"" << GetContext() + GetId() << "\"");
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
                  /* Identifer not found in the collection of components with the specified base type -> error */
                  THROW_ARGOSEXCEPTION("There is no component of type \"" << str_component.substr(0,unIdentifierStart) << "\" in entity \"" << GetContext() + GetId() << "\" with an Id \"" << strComponentId << "\".");
               }
            }
         }
         else {
            THROW_ARGOSEXCEPTION("Syntax error in entity id \"" << str_component << "\"");
         }
      }
      else {
         /* No [ found in the string, consider the entire string as a type id */
         /* Count how many elements match the given type id */
         size_t unCount = m_mapComponents.count(str_component);
         if(unCount == 0) {
            /* No components -> error */
            THROW_ARGOSEXCEPTION("No component of type \"" << str_component << "\" found for entity \"" << GetContext() + GetId() << "\"");
         }
         else if(unCount > 1) {
            /* Not used array syntax, but needed to -> error */
            THROW_ARGOSEXCEPTION("You need to provide an index for component of type \"" << str_component.substr(0,unIdentifierStart) << "\" in entity \"" << GetContext() + GetId() << "\": " << unCount << " matching elements are present");
         }
         else {
            /* All OK, return the element */
            return m_mapComponents.find(str_component);
         }
      }
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_SPACE_OPERATIONS_ON_COMPOSABLE(CComposableEntity);

   /****************************************/
   /****************************************/

}
