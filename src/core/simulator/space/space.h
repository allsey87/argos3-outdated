/**
 * @file <argos3/core/simulator/space/space.h>
 *
 * @brief This file provides the definition of the space.
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 */

#ifndef SPACE_H
#define SPACE_H

namespace argos {
   class CSpace;
   class CRay3;
   class CFloorEntity;
}

#include <argos3/core/utility/datatypes/any.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/entity/controllable_entity.h>
#include <argos3/core/simulator/entity/medium_entity.h>
#include <argos3/core/simulator/entity/led_entity.h>
#include <argos3/core/simulator/entity/rab_equipped_entity.h>

namespace argos {

   /****************************************/
   /****************************************/

   class CSpace : public CBaseConfigurableResource {

   public:

      /** Maps for quick access to physical entities (robots, objects) */
      typedef std::map <std::string, CAny, std::less <std::string> > TMapPerType;
      typedef std::map <std::string, TMapPerType, std::less <std::string> > TMapPerTypePerId;

      template <class E> struct SEntityIntersectionItem {
         E* IntersectedEntity;
         Real TOnRay;

         SEntityIntersectionItem() :
            IntersectedEntity(NULL),
            TOnRay(0.0f) {}

         SEntityIntersectionItem(E* pc_entity, Real f_t_on_ray) :
            IntersectedEntity(pc_entity),
            TOnRay(f_t_on_ray) {}

         inline bool operator<(const SEntityIntersectionItem& s_item) {
            return TOnRay < s_item.TOnRay;
         }
      };

      template <class E> struct SEntityIntersectionData {
         bool Intersection;
         std::vector<SEntityIntersectionItem<E>*> IntersectedEntities;

         SEntityIntersectionData() :
            Intersection(false) {}

         SEntityIntersectionData(std::vector<SEntityIntersectionItem<E>*>& c_entities) :
            Intersection(c_entities.size() > 0),
            IntersectedEntities(c_entities) {}
      };

   protected:

      class CRayEmbodiedEntityIntersectionMethod;
      class CRayEmbodiedEntityIntersectionSpaceHash;
      class CRayEmbodiedEntityIntersectionEntitySweep;

      /****************************************/
      /****************************************/

   public:

      /**
       * Class constructor.
       */
      CSpace();

      /**
       * Class destructor.
       */
      virtual ~CSpace() {}

      /**
       * Initializes the space using the <tt>&lt;arena&gt;</tt> section of the XML configuration file.
       * @param t_tree the <tt>&lt;arena&gt;</tt> section of the XML configuration file.
       */
      virtual void Init(TConfigurationNode& t_tree);

      /**
       * Reset the space and all its entities.
       */
      virtual void Reset();

      /**
       * Destroys the space and all its entities.
       */
      virtual void Destroy();

      /**
       * Returns the number of entities contained in the space.
       */
      inline UInt32 GetNumberEntities() const {
         return m_vecEntities.size();
      }

      /**
       * Returns a vector of all the entities in the space.
       * All entities are returned, i.e., all the components of a robot.
       * @return a vector of all the entities in the space.
       * @see GetRootEntityVector()
       */
      inline CEntity::TVector& GetEntityVector() {
         return m_vecEntities;
      }

      /**
       * Returns a vector of all the root entities in the space.
       * A root entity is an entity that has no parent.
       * This method differs from GetEntityVector() in that the latter
       * returns all entities including the components of a composable
       * entity, while this method does not return any component, but only
       * the parentless composables.
       * @return a vector of all the root entities in the space.
       * @see GetEntityVector()
       */
      inline CEntity::TVector& GetRootEntityVector() {
         return m_vecRootEntities;
      }

      /**
       * Returns the entity with the given id.
       * @param str_id The id of the wanted entity
       * @return The entity with the given id.       
       * @throws CARGoSException if an entity with the wanted id does not exist
       */
      inline CEntity& GetEntity(const std::string& str_id) {
         CEntity::TMap::const_iterator it = m_mapEntitiesPerId.find(str_id);
         if ( it != m_mapEntitiesPerId.end()) {
            return *(it->second);
         }
         THROW_ARGOSEXCEPTION("Unknown entity id \"" << str_id <<
                              "\" when requesting entity from space.");
      }

      /**
       * Returns the entities matching a given pattern.
       * The pattern must be a valid regexp.
       * @param t_buffer A vector filled with all the entities that match the given pattern.
       * @param str_pattern The pattern to match.
       * @return The entity with the given id.       
       * @throws CARGoSException if the regexp is not valid.
       */
      void GetEntitiesMatching(CEntity::TVector& t_buffer,
                               const std::string& str_pattern);

      /**
       * Calculates the closest intersection point along the given ray.
       * @param s_data Buffer containing the calculated intersection point, or lack thereof.
       * @param c_ray The test ray.
       * @param set_ignored_entities A list of entities that must be ignored if an intersection with them is detected.
       */
      bool GetClosestEmbodiedEntityIntersectedByRay(SEntityIntersectionItem<CEmbodiedEntity>& s_data,
                                                    const CRay3& c_ray,
                                                    const TEmbodiedEntitySet& set_ignored_entities = TEmbodiedEntitySet());

      /**
       * Returns a map of all entities ordered by id.
       * @return a map of all entities ordered by id.
       */
      inline CEntity::TMap& GetEntityMapPerId() {
         return m_mapEntitiesPerId;
      }

      /**
       * Returns a nested map of entities, ordered by type and by id.
       * The 'type' here refers to the string returned by CEntity::GetTypeDescription().
       * Take this example:
       * <code>
       *    CSpace::TMapPerTypePerId& theMap = space.GetEntityMapPerTypePerId();
       *    // theMap["box"] is a CSpace::TMapPerType containing all the box entities, ordered by id
       *    // theMap["led"] is a CSpace::TMapPerType containing all the led entities, ordered by id
       *    // etc.
       *    CBoxEntity* box = any_cast<CBoxEntity*>(theMap["box"]["my_box_22"]);
       *    // do stuff with the box ...
       * </code>
       * @returns a nested map of entities, ordered by type and by id.
       * @see CEntity::GetTypeDescription()
       * @see TMapPerType
       * @see GetEntitiesByType()
       */
      inline TMapPerTypePerId& GetEntityMapPerTypePerId() {
         return m_mapEntitiesPerTypePerId;
      }

      /**
       * Returns a map containing all the objects of a given type.
       * The 'type' here refers to the string returned by CEntity::GetTypeDescription().
       * Take this example:
       * <code>
       *    CSpace::TMapPerType& theMap = space.GetEntityMapByType("box");
       *    CBoxEntity* box = any_cast<CBoxEntity*>(theMap["my_box"]);
       *    // do stuff with the box ...
       * </code>
       * @param str_type The wanted type to search for.
       * @return A map containing all the objects of a given type.
       * @throws CARGoSException if the given type is not valid.
       * @see CEntity::GetTypeDescription()
       * @see TMapPerType
       * @see GetEntityMapPerTypePerId()
       */
      TMapPerType& GetEntitiesByType(const std::string& str_type);

      /**
       * Returns the floor entity.
       * @throws CARGoSException if the floor entity has not been added to the arena.
       * @return The floor entity.
       */
      inline CFloorEntity& GetFloorEntity() {
         if(m_pcFloorEntity != NULL) return *m_pcFloorEntity;
         else THROW_ARGOSEXCEPTION("No floor entity has been added to the arena.");
      }

      /**
       * Sets the floor entity.
       * @param c_floor_entity The floor entity.
       */
      inline void SetFloorEntity(CFloorEntity& c_floor_entity) {
         m_pcFloorEntity = &c_floor_entity;
      }

      /**
       * Returns <tt>true</tt> if positional entities are indexed using the space hash.
       * @return <tt>true</tt> if positional entities are indexed using the space hash.
       */
      inline bool IsUsingSpaceHash() const {
         return m_bUseSpaceHash;
      }

      /**
       * Sets the list of physics engines.
       * This method is used internally.
       */
      inline virtual void SetPhysicsEngines(CPhysicsEngine::TVector& t_engines) {
         m_ptPhysicsEngines = &t_engines;
      }

      /**
       * Updates the space.
       */
      virtual void Update();

      /**
       * Adds an entity of the given type.
       * This method is used internally, don't use it in your code.
       * throws CARGoSException if the entity id already exists in the space indexes.
       */
      template <typename ENTITY>
      void AddEntity(ENTITY& c_entity) {
         /* Check that the id of the entity is not already present */
         if(m_mapEntitiesPerId.find(c_entity.GetId()) != m_mapEntitiesPerId.end()) {
            THROW_ARGOSEXCEPTION("Error inserting a " << c_entity.GetTypeDescription() << " entity with id \"" << c_entity.GetId() << "\". An entity with that id exists already.");
         }
         /* Add the entity to the indexes */
         if(!c_entity.HasParent()) {
            m_vecRootEntities.push_back(&c_entity);
         }
         m_vecEntities.push_back(&c_entity);
         m_mapEntitiesPerId[c_entity.GetId()] = &c_entity;
         m_mapEntitiesPerTypePerId[c_entity.GetTypeDescription()][c_entity.GetId()] = &c_entity;
      }

      /**
       * Removes an entity of the given type.
       * This method is used internally, don't use it in your code.
       * throws CARGoSException if the entity id does not exist in the space indexes.
       */
      template <typename ENTITY>
      void RemoveEntity(ENTITY& c_entity) {
         /* Search for entity in the index per type */
         TMapPerTypePerId::iterator itMapPerType = m_mapEntitiesPerTypePerId.find(c_entity.GetTypeDescription());
         if(itMapPerType != m_mapEntitiesPerTypePerId.end()) {
            /* Search for entity in the index per type per id */
            TMapPerType::iterator itMapPerTypePerId = itMapPerType->second.find(c_entity.GetId());
            if(itMapPerTypePerId != itMapPerType->second.end()) {
               /* Remove the entity from the indexes */
               CEntity::TVector::iterator itVec = find(m_vecEntities.begin(),
                                                       m_vecEntities.end(),
                                                       &c_entity);
               m_vecEntities.erase(itVec);
               CEntity::TMap::iterator itMap = m_mapEntitiesPerId.find(c_entity.GetId());
               itMapPerType->second.erase(itMapPerTypePerId);
               m_mapEntitiesPerId.erase(itMap);
               if(!c_entity.HasParent()) {
                  CEntity::TVector::iterator itRootVec = find(m_vecRootEntities.begin(),
                                                              m_vecRootEntities.end(),
                                                              &c_entity);
                  m_vecRootEntities.erase(itRootVec);
               }
               /* Remove entity object */
               c_entity.Destroy();
               delete &c_entity;
               return;
            }
         }
         THROW_ARGOSEXCEPTION("CSpace::RemoveEntity() : Entity \"" <<
                              c_entity.GetId() <<
                              "\" has not been found in the indexes.");
      }

      /**
       * Returns the current value of the simulation clock.
       * The clock is measured in ticks. You can set how much a tick is long in seconds in the XML.
       * @return The current value of the simulation clock.
       */
      inline UInt32 GetSimulationClock() const {
         return m_unSimulationClock;
      }

      /**
       * Sets a new value for the simulation clock.
       * The clock is measured in ticks. You can set how much a tick is long in seconds in the XML.
       * @param un_simulation_clock The new value for the simulation clock.
       */
      inline void SetSimulationClock(UInt32 un_simulation_clock) {
         m_unSimulationClock = un_simulation_clock;
      }

      /**
       * Increases the simulation clock by the wanted value.
       * The clock is measured in ticks. You can set how much a tick is long in seconds in the XML.
       * @param un_increase The quantity to add to the current value of the simulation clock.
       */
      inline void IncreaseSimulationClock(UInt32 un_increase = 1) {
         m_unSimulationClock += un_increase;
      }

      /**
       * Returns the arena size.
       * @return the arena size.
       */
      inline const CVector3& GetArenaSize() const {
         return m_cArenaSize;
      }

      /**
       * Sets the arena size.
       * @return the arena size.
       */
      inline void SetArenaSize(const CVector3& c_size) {
         m_cArenaSize = c_size;
      }

      /**
       * Returns the space hash containing the embodied entities.
       * @return The space hash containing the embodied entities.
       * @throw CARGoSException if the space hash is not being used.
       */
      inline CSpaceHash<CEmbodiedEntity, CEmbodiedEntitySpaceHashUpdater>& GetEmbodiedEntitiesSpaceHash() {
         if(IsUsingSpaceHash()) {
            return *m_pcEmbodiedEntitiesSpaceHash;
         }
         else {
            THROW_ARGOSEXCEPTION("Attempted to access the space hash of embodied entities, but in the XML the user chose not to use it. Maybe you use a sensor or an actuator that references it directly?");
         }
      }

      /**
       * Returns the space hash containing the LED entities.
       * @return The space hash containing the LED entities.
       * @throw CARGoSException if the space hash is not being used.
       */
      inline CSpaceHash<CLEDEntity, CLEDEntitySpaceHashUpdater>& GetLEDEntitiesSpaceHash() {
         if(IsUsingSpaceHash()) {
            return *m_pcLEDEntitiesSpaceHash;
         }
         else {
            THROW_ARGOSEXCEPTION("Attempted to access the space hash of LED entities, but in the XML the user chose not to use it. Maybe you use a sensor or an actuator that references it directly?");
         }
      }

      /**
       * Returns the space hash containing the RAB equipped entities.
       * @return The space hash containing the RAB equipped entities.
       * @throw CARGoSException if the space hash is not being used.
       */
      inline CSpaceHash<CRABEquippedEntity, CRABEquippedEntitySpaceHashUpdater>& GetRABEquippedEntitiesSpaceHash() {
         if(IsUsingSpaceHash()) {
            return *m_pcRABEquippedEntitiesSpaceHash;
         }
         else {
            THROW_ARGOSEXCEPTION("Attempted to access the space hash of RAB equipped entities, but in the XML the user chose not to use it. Maybe you use a sensor or an actuator that references it directly?");
         }
      }

   protected:

      virtual void AddControllableEntity(CControllableEntity& c_entity);
      virtual void RemoveControllableEntity(CControllableEntity& c_entity);
      virtual void AddMediumEntity(CMediumEntity& c_entity);
      virtual void RemoveMediumEntity(CMediumEntity& c_entity);
      virtual void AddEntityToPhysicsEngine(CEmbodiedEntity& c_entity);
      
      void UpdateSpaceData();
      
      virtual void UpdateControllableEntities() = 0;
      virtual void UpdatePhysics() = 0;
      
      void UpdateMediumEntities();

      void Distribute(TConfigurationNode& t_tree);

      void AddBoxStrip(TConfigurationNode& t_tree);

      bool GetClosestEmbodiedEntityIntersectedByRaySpaceHash(SEntityIntersectionItem<CEmbodiedEntity>& s_data,
                                                             const CRay3& c_ray,
                                                             const TEmbodiedEntitySet& set_ignored_entities);

      bool GetClosestEmbodiedEntityIntersectedByRayEntitySweep(SEntityIntersectionItem<CEmbodiedEntity>& s_data,
                                                               const CRay3& c_ray,
                                                               const TEmbodiedEntitySet& set_ignored_entities);

   protected:

      friend class CSpaceOperationAddControllableEntity;
      friend class CSpaceOperationRemoveControllableEntity;
      friend class CSpaceOperationAddMediumEntity;
      friend class CSpaceOperationRemoveMediumEntity;
      friend class CSpaceOperationAddEmbodiedEntity;

   protected:

      /** The current simulation clock */
      UInt32 m_unSimulationClock;

      /** Arena size */
      CVector3 m_cArenaSize;

      /** A vector of entities. */
      CEntity::TVector m_vecEntities;

      /** A vector of all the entities without a parent */
      CEntity::TVector m_vecRootEntities;

      /** A map of entities. */
      CEntity::TMap m_mapEntitiesPerId;

      /** A map of maps of all the simulated entities.
          The top-level map is indexed by type, as returned by CEntity::GetTypeDescription().
          The second-level maps are indexed by entity id */
      TMapPerTypePerId m_mapEntitiesPerTypePerId;

      /** The space hash of embodied entities */
      CSpaceHash<CEmbodiedEntity, CEmbodiedEntitySpaceHashUpdater>* m_pcEmbodiedEntitiesSpaceHash;

      /** The space hash of LED entities */
      CSpaceHash<CLEDEntity, CLEDEntitySpaceHashUpdater>* m_pcLEDEntitiesSpaceHash;

      /** The space hash of RAB equipped entities */
      CSpaceHash<CRABEquippedEntity, CRABEquippedEntitySpaceHashUpdater>* m_pcRABEquippedEntitiesSpaceHash;

      /** A vector of controllable entities */
      CControllableEntity::TVector m_vecControllableEntities;

      /** A vector of medium entities */
      CMediumEntity::TVector m_vecMediumEntities;

      /** The floor entity */
      CFloorEntity* m_pcFloorEntity;

      /** True if the space hash should be used */
      bool m_bUseSpaceHash;

      /** Method to calculate the ray-embodied entity intersection */
      CRayEmbodiedEntityIntersectionMethod* m_pcRayEmbodiedEntityIntersectionMethod;

      /** A reference to the list of physics engines */
      CPhysicsEngine::TVector* m_ptPhysicsEngines;
   };

   /****************************************/
   /****************************************/

   template <typename ACTION>
   class CSpaceOperation : public CEntityOperation<ACTION, CSpace, void> {
   public:
      virtual ~CSpaceOperation() {}
   };

   class CSpaceOperationAddEntity : public CSpaceOperation<CSpaceOperationAddEntity> {
   public:
      virtual ~CSpaceOperationAddEntity() {}
   };
   class CSpaceOperationRemoveEntity : public CSpaceOperation<CSpaceOperationRemoveEntity> {
   public:
      virtual ~CSpaceOperationRemoveEntity() {}
   };

}

   /****************************************/
   /****************************************/

#define SPACE_OPERATION_ADD_ENTITY(ENTITY)                                 \
   class CSpaceOperationAdd ## ENTITY : public CSpaceOperationAddEntity {  \
   public:                                                                 \
      void ApplyTo(CSpace& c_space, ENTITY& c_entity) {                    \
         c_space.AddEntity(c_entity);                                      \
      }                                                                    \
   };

#define SPACE_OPERATION_REMOVE_ENTITY(ENTITY)                                   \
   class CSpaceOperationRemove ## ENTITY : public CSpaceOperationRemoveEntity { \
   public:                                                                      \
      void ApplyTo(CSpace& c_space, ENTITY& c_entity) {                         \
         c_space.RemoveEntity(c_entity);                                        \
      }                                                                         \
   };

#define REGISTER_SPACE_OPERATION(ACTION, OPERATION, ENTITY)             \
   REGISTER_ENTITY_OPERATION(ACTION, CSpace, OPERATION, void, ENTITY);

#define REGISTER_STANDARD_SPACE_OPERATION_ADD_ENTITY(ENTITY)            \
   SPACE_OPERATION_ADD_ENTITY(ENTITY)                                   \
   REGISTER_SPACE_OPERATION(CSpaceOperationAddEntity,                   \
                            CSpaceOperationAdd ## ENTITY,               \
                            ENTITY);

#define REGISTER_STANDARD_SPACE_OPERATION_REMOVE_ENTITY(ENTITY)         \
   SPACE_OPERATION_REMOVE_ENTITY(ENTITY)                                \
   REGISTER_SPACE_OPERATION(CSpaceOperationRemoveEntity,                \
                            CSpaceOperationRemove ## ENTITY,            \
                            ENTITY);

#define REGISTER_STANDARD_SPACE_OPERATIONS_ON_ENTITY(ENTITY) \
   REGISTER_STANDARD_SPACE_OPERATION_ADD_ENTITY(ENTITY)      \
   REGISTER_STANDARD_SPACE_OPERATION_REMOVE_ENTITY(ENTITY)

#define REGISTER_STANDARD_SPACE_OPERATION_ADD_COMPOSABLE(ENTITY)        \
   SPACE_OPERATION_ADD_COMPOSABLE_ENTITY(ENTITY)                        \
   REGISTER_SPACE_OPERATION(CSpaceOperationAddEntity,                   \
                            CSpaceOperationAdd ## ENTITY,               \
                            ENTITY);

#define REGISTER_STANDARD_SPACE_OPERATION_REMOVE_COMPOSABLE(ENTITY)        \
   SPACE_OPERATION_REMOVE_COMPOSABLE_ENTITY(ENTITY)                        \
   REGISTER_SPACE_OPERATION(CSpaceOperationRemoveEntity,                   \
                            CSpaceOperationRemove ## ENTITY,               \
                            ENTITY);

#define REGISTER_STANDARD_SPACE_OPERATIONS_ON_COMPOSABLE(ENTITY)  \
   REGISTER_STANDARD_SPACE_OPERATION_ADD_COMPOSABLE(ENTITY)       \
   REGISTER_STANDARD_SPACE_OPERATION_REMOVE_COMPOSABLE(ENTITY)

#endif
