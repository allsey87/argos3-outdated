/**
 * @file <argos3/core/simulator/entity/entity.h>
 *
 * @brief This file contains the definition of an entity.
 *
 * This file contains the class definition of an
 * entity, that is, the basic class that provides the interface for the
 * simulation of all the objects in the environment.
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 */

#ifndef ENTITY_H
#define ENTITY_H

namespace argos {
   class CEntity;
   class CComposableEntity;
   class CSpace;
}

#include <argos3/core/utility/datatypes/datatypes.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/configuration/base_configurable_resource.h>
#include <argos3/core/utility/plugins/factory.h>
#include <argos3/core/utility/plugins/vtable.h>

#include <vector>
#include <tr1/unordered_map>
#include <string>

namespace argos {

   class CEntity : public CBaseConfigurableResource,
                   public EnableVTableFor<CEntity> {

   public:

      ENABLE_VTABLE();

      typedef std::vector<CEntity*> TVector;
      typedef std::tr1::unordered_map<std::string, CEntity*> TMap;
      typedef std::tr1::unordered_multimap<std::string, CEntity*> TMultiMap;

   public:

      CEntity(CComposableEntity* pc_parent);

      CEntity(CComposableEntity* pc_parent,
              const std::string& str_id);

      virtual ~CEntity() {}

      virtual void Init(TConfigurationNode& t_tree);

      virtual void Reset() {}

      virtual void Destroy() {}

      inline virtual const std::string& GetId() const {
         return m_strId;
      }

      inline bool HasParent() {
         return (m_pcParent != NULL);
      }

      inline CComposableEntity& GetParent() {
         return *m_pcParent;
      }

      inline void SetParent(CComposableEntity& c_parent) {
         m_pcParent = &c_parent;
      }

      virtual std::string GetTypeDescription() const {
         return "entity";
      }

      virtual void Update() {}

      /**
       * Returns <tt>true</tt> if the entity is enabled
       * @return <tt>true</tt> if the entity is enabled
       * @see m_bEnabled
       * @see m_bCanBeEnabledIfDisabled
       */
      bool IsEnabled() const {
         return m_bEnabled;
      }

      /**
       * Enables or disables an entity
       * @param b_enabled <tt>true</tt> if the entity is enabled, <tt>false</tt> otherwise
       * @see m_bEnabled
       * @see m_bCanBeEnabledIfDisabled
       */
      virtual void SetEnabled(bool b_enabled);

      /**
       * Returns <tt>true</tt> if this entity, once disabled, can be re-enabled
       * @return <tt>true</tt> if this entity, once disabled, can be re-enabled
       * @see m_bEnabled
       * @see m_bCanBeEnabledIfDisabled
       */
      bool CanBeEnabledIfDisabled() const {
         return m_bCanBeEnabledIfDisabled;
      }

      /**
       * Sets whether this entity, once disabled, can be re-enabled
       * @return <tt>true</tt> if this entity, once disabled, can be re-enabled; <tt>false</tt> otherwise
       * @see m_bEnabled
       * @see m_bCanBeEnabledIfDisabled
       */
      void SetCanBeEnabledIfDisabled(bool b_can_it) {
         m_bCanBeEnabledIfDisabled = b_can_it;
      }

   private:

      /** The parent of this entity */
      CComposableEntity* m_pcParent;

      /** The id of this entity */
      std::string m_strId;

      /** When <tt>true</tt>, this entity is updated; when <tt>false</tt>, this entity is not updated */
      bool m_bEnabled;

      /**
       * This flag is used when an entity is disabled.
       * If it is set to <tt>true</tt>, it means that the entity can be re-enabled;
       * when set to <tt>false</tt>, the entity stays disabled even if SetEnabled(<tt>true</tt>)
       * is called.
       */
      bool m_bCanBeEnabledIfDisabled;

   };

   /**
    * The basic operation to be stored in the vtable
    */
   template <typename LABEL, typename PLUGIN, typename RETURN_TYPE>
   class CEntityOperation {
   public:
      template <typename DERIVED, typename OPERATION_IMPL>
      RETURN_TYPE Hook(PLUGIN& t_plugin, CEntity& c_entity) {
         return Dispatch<DERIVED, OPERATION_IMPL>(t_plugin, c_entity);
      }
   protected:
      template <typename DERIVED, typename OPERATION_IMPL>
      RETURN_TYPE Dispatch(PLUGIN& t_plugin, CEntity& c_entity) {
         /* First dispatch: cast this operation into the specific operation */
         OPERATION_IMPL& tOperation = static_cast<OPERATION_IMPL&>(*this);
         /* Second dispatch: cast t_base to DERIVED */
         DERIVED& tDerived = static_cast<DERIVED&>(c_entity);
         /* Perform visit */
         return tOperation.ApplyTo(t_plugin, tDerived);
      }
   };

   template <typename LABEL, typename PLUGIN, typename RETURN_TYPE>
   class CEntityOperationInstanceHolder {
   public:
      ~CEntityOperationInstanceHolder() {
         while(!m_vecOperationInstances.empty()) {
            if(m_vecOperationInstances.back() != NULL) {
               delete m_vecOperationInstances.back();
            }
            m_vecOperationInstances.pop_back();
         }
      }
      template <typename DERIVED>
      void Add(CEntityOperation<LABEL, PLUGIN, RETURN_TYPE>* pc_operation) {
         /* Find the slot */
         size_t unIndex = GetTag<DERIVED, CEntity>();
         /* Does the holder have a slot for this index? */
         if(unIndex >= m_vecOperationInstances.size()) {
            /* No, new slots must be created
             * Fill the slots with NULL
             */
            /* Create new slots up to index+1 and fill them with tDefaultFunction */
            m_vecOperationInstances.resize(unIndex+1, NULL);
         }
         m_vecOperationInstances[unIndex] = pc_operation;
      }
      CEntityOperation<LABEL, PLUGIN, RETURN_TYPE>* operator[](size_t un_index) const {
         if(un_index >= m_vecOperationInstances.size()) {
            return NULL;
         }
         return m_vecOperationInstances[un_index];
      }
   private:
      std::vector<CEntityOperation<LABEL, PLUGIN, RETURN_TYPE>*> m_vecOperationInstances;
   };

   /**
    * Function that returns a reference to the static operation instance holder
    */
   template <typename LABEL, typename PLUGIN, typename RETURN_VALUE>
   CEntityOperationInstanceHolder<LABEL, PLUGIN, RETURN_VALUE>& GetEntityOperationInstanceHolder() {
      static CEntityOperationInstanceHolder<LABEL, PLUGIN, RETURN_VALUE> cHolder;
      return cHolder;
   }

   /**
    * Calls the operation corresponding to the given context and operand
    * Skips the function call if the operation is missing in the vtable
    */
   template<typename LABEL, typename PLUGIN, typename RETURN_VALUE>
   RETURN_VALUE CallEntityOperation(PLUGIN& t_plugin, CEntity& c_entity) {
      typedef RETURN_VALUE (CEntityOperation<LABEL, PLUGIN, RETURN_VALUE>::*TFunction)(PLUGIN& t_plugin, CEntity&);
      TFunction tFunction = GetVTable<LABEL, CEntity, TFunction>()[c_entity.GetTag()];
      if(tFunction != NULL) {
         CEntityOperation<LABEL, PLUGIN, RETURN_VALUE>* pcOperation = GetEntityOperationInstanceHolder<LABEL, PLUGIN, RETURN_VALUE>()[c_entity.GetTag()];
         if(pcOperation != NULL) {
            return (pcOperation->*tFunction)(t_plugin, c_entity);
         }
      }
      return RETURN_VALUE();
   }

}

#define REGISTER_ENTITY(CLASSNAME,                  \
                        LABEL,                      \
                        AUTHOR,                     \
                        VERSION,                    \
                        BRIEF_DESCRIPTION,          \
                        LONG_DESCRIPTION,           \
                        STATUS)                     \
   REGISTER_SYMBOL(CEntity,                         \
                   CLASSNAME,                       \
                   LABEL,                           \
                   AUTHOR,                          \
                   VERSION,                         \
                   BRIEF_DESCRIPTION,               \
                   LONG_DESCRIPTION,                \
                   STATUS)

/**
 * Convenience macro to register vtable entity operations
 */
#define REGISTER_ENTITY_OPERATION(LABEL, PLUGIN, OPERATION, RETURN_VALUE, DERIVED)                         \
   class C ## LABEL ## PLUGIN ## OPERATION ## RETURN_VALUE ## DERIVED {                                    \
      typedef RETURN_VALUE (CEntityOperation<LABEL, PLUGIN, RETURN_VALUE>::*TFunction)(PLUGIN&, CEntity&); \
   public:                                                                                                 \
      C ## LABEL ## PLUGIN ## OPERATION ## RETURN_VALUE ## DERIVED() {                                     \
         GetVTable<LABEL, CEntity, TFunction>().Add<DERIVED>(&OPERATION::Hook<DERIVED, OPERATION>);        \
         GetEntityOperationInstanceHolder<LABEL, PLUGIN, RETURN_VALUE>().Add<DERIVED>(new OPERATION());    \
      }                                                                                                    \
   } c ## LABEL ## PLUGIN ## OPERATION ## RETURN_VALUE ## DERIVED;

#endif
