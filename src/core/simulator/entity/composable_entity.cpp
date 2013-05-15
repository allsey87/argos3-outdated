
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

   /*
   CEntity& CComposableEntity::GetComponent(const std::string& str_component) {
      try {
         return *(FindComponent(str_component)->second);
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("While getting a component from a composable entity", ex);
      }
   }
   */

   CEntity& CComposableEntity::GetComponent(const std::string& str_path) {
      try {
         fprintf(stderr, "%s%s::GetComponent(\"%s\")\n", GetContext().c_str(), GetId().c_str(), str_path.c_str());
         size_t unFirstSeperatorIdx = str_path.find("/");
         std::string strFrontIdentifier = str_path.substr(0, unFirstSeperatorIdx);
         fprintf(stderr, "strFrontIdentifier = %s\n", strFrontIdentifier.c_str());
         CEntity* pcEntity = FindComponent(strFrontIdentifier)->second;
         if(unFirstSeperatorIdx == std::string::npos) {
            fprintf(stderr, "we contain this component, returning...\n");
            return *pcEntity;
         }
         else {
            fprintf(stderr, "we contain don't have this component, doing recusive lookup...\n");
            CComposableEntity* pcComposableEntity = dynamic_cast<CComposableEntity*>(pcEntity);
            if(pcComposableEntity != NULL) {
               fprintf(stderr, "conversion to composable sucessfully, requesting %s from %s\n", str_path.substr(unFirstSeperatorIdx + 1, std::string::npos).c_str(), strFrontIdentifier.c_str());
               return pcComposableEntity->GetComponent(str_path.substr(unFirstSeperatorIdx + 1, std::string::npos));
            }
            else {
               THROW_ARGOSEXCEPTION("Component \"" << strFrontIdentifier << "\" of \"" << GetContext() << GetId()
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
      
      fprintf(stderr, "executing %s%s::FindComponent(%s)\n", GetContext().c_str(), GetId().c_str(), str_component.c_str());
      /* Check for the presence of [ */
      std::string::size_type unIndexStart = str_component.find('[');
      if(unIndexStart != std::string::npos) {
         /* Found, now check for the presence of ] after [ */
         std::string::size_type unIndexEnd = str_component.rfind(']');
         if(unIndexEnd != std::string::npos &&
            unIndexEnd > unIndexStart) {
            /* Use the string between [ and ] as an index and whatever comes before as base id */
            /* Count how many components there are for the base type */
            std::string strBaseType = str_component.substr(0,unIndexStart);
            ///////////// DEBUG

            fprintf(stderr, "searching for strBaseType %s in m_mapComponents\n it contains:\n", strBaseType.c_str());

            for(CEntity::TMultiMap::iterator it = m_mapComponents.begin();
                it != m_mapComponents.end();
                ++it) {
               fprintf(stderr, "%s\n", it->first.c_str());
            }

            /////////// DEBUG END
            size_t unCount = m_mapComponents.count(strBaseType);
            fprintf(stderr, "we have %lu of component type %s\n", unCount, strBaseType.c_str());
            if(unCount == 0) {
               /* No components -> error */
               THROW_ARGOSEXCEPTION("No component of type \"" << strBaseType << "\" found for entity \"" << GetId() << "\"");
            }
            else {
               /* Components found */
               //size_t unIndex = FromString<size_t>(str_component.substr(unIndexStart + 1, unIndexEnd - unIndexStart));

               std::string strComponentId = str_component.substr(unIndexStart + 1, unIndexEnd - unIndexStart - 1);

               //fprintf(stderr, "the index %s, was decoded as  %lu\n", str_component.substr(unIndexStart + 1, unIndexEnd - unIndexStart).c_str(), unIndex);
               /* Is index valid? */
               std::pair<CEntity::TMultiMap::iterator,
                         CEntity::TMultiMap::iterator> cRange = m_mapComponents.equal_range(strBaseType);
               
               CEntity::TMultiMap::iterator itComponent;

               for(itComponent = cRange.first;
                   itComponent != cRange.second;
                   ++itComponent) {
                  if(itComponent->second->GetId() == strComponentId) {
                     break;
                  }
               }

               if(itComponent != cRange.second) {
                  return itComponent;
               }
               else {
                  /* Index out of bounds -> error */
                  THROW_ARGOSEXCEPTION("There is no component of type \"" << str_component.substr(0,unIndexStart) << "\" in entity \"" << GetContext() << GetId() << "\" with an Id \"" << strComponentId << "\".");
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
            THROW_ARGOSEXCEPTION("No component of type \"" << str_component << "\" found for entity \"" << GetId() << "\"");
         }
         else if(unCount > 1) {
            /* Not used array syntax, but needed to -> error */
            THROW_ARGOSEXCEPTION("You need to provide an index for component of type \"" << str_component.substr(0,unIndexStart) << "\" in entity \"" << GetId() << "\": " << unCount << " matching elements are present");
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
