/**
 * @file <argos3/core/simulator/space/space.cpp>
 *
 * @brief This file provides the implementation of the space.
 *
 * This file provides the implementation of the space.
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 */

#include <argos3/core/utility/string_utilities.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space_hash_native.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <cstring>
#include "space.h"

namespace argos {

   /****************************************/
   /****************************************/

   class CSpace::CRayEmbodiedEntityIntersectionMethod {
   public:
      CRayEmbodiedEntityIntersectionMethod(CSpace& c_space) : m_cSpace(c_space) {}
      virtual ~CRayEmbodiedEntityIntersectionMethod() {}
      virtual bool operator()(SEntityIntersectionItem<CEmbodiedEntity>& s_data,
                              const CRay3& c_ray,
                              const TEmbodiedEntitySet& set_ignored_entities) = 0;
   protected:
      CSpace& m_cSpace;
   };

   class CSpace::CRayEmbodiedEntityIntersectionSpaceHash : public CSpace::CRayEmbodiedEntityIntersectionMethod {
   public:
      CRayEmbodiedEntityIntersectionSpaceHash(CSpace& c_space) : CSpace::CRayEmbodiedEntityIntersectionMethod(c_space) {}
      virtual bool operator()(CSpace::SEntityIntersectionItem<CEmbodiedEntity>& s_data,
                              const CRay3& c_ray,
                              const TEmbodiedEntitySet& set_ignored_entities) {
         return m_cSpace.GetClosestEmbodiedEntityIntersectedByRaySpaceHash(s_data, c_ray, set_ignored_entities);
      }
   };

   class CSpace::CRayEmbodiedEntityIntersectionEntitySweep : public CSpace::CRayEmbodiedEntityIntersectionMethod {
   public:
      CRayEmbodiedEntityIntersectionEntitySweep(CSpace& c_space) : CSpace::CRayEmbodiedEntityIntersectionMethod(c_space) {}
      virtual bool operator()(CSpace::SEntityIntersectionItem<CEmbodiedEntity>& s_data,
                              const CRay3& c_ray,
                              const TEmbodiedEntitySet& set_ignored_entities) {
         return m_cSpace.GetClosestEmbodiedEntityIntersectedByRayEntitySweep(s_data, c_ray, set_ignored_entities);
      }
   };

   /****************************************/
   /****************************************/

   CSpace::CSpace() :
      m_unSimulationClock(0),
      m_pcEmbodiedEntitiesSpaceHash(NULL),
      m_pcLEDEntitiesSpaceHash(NULL),
      m_pcRABEquippedEntitiesSpaceHash(NULL),
      m_pcFloorEntity(NULL),
      m_bUseSpaceHash(true),
      m_pcRayEmbodiedEntityIntersectionMethod(NULL),
      m_ptPhysicsEngines(NULL) {
   }
   
   /****************************************/
   /****************************************/

   void CSpace::Init(TConfigurationNode& t_tree) {
      /* Get the arena size */
      GetNodeAttribute(t_tree, "size", m_cArenaSize);
      /* Get the list of physics engines */
      m_ptPhysicsEngines = &(CSimulator::GetInstance().GetPhysicsEngines());
      /* Should we use the space hash, and, if so, which one? */
      GetNodeAttributeOrDefault(t_tree, "hashing", m_bUseSpaceHash, true);
      if(m_bUseSpaceHash) {
         m_pcRayEmbodiedEntityIntersectionMethod = new CRayEmbodiedEntityIntersectionSpaceHash(*this);
         m_pcEmbodiedEntitiesSpaceHash = new CSpaceHashNative<CEmbodiedEntity, CEmbodiedEntitySpaceHashUpdater>;
         m_pcLEDEntitiesSpaceHash = new CSpaceHashNative<CLEDEntity, CLEDEntitySpaceHashUpdater>;
         m_pcRABEquippedEntitiesSpaceHash = new CSpaceHashNative<CRABEquippedEntity, CRABEquippedEntitySpaceHashUpdater>;
      }
      else {
         LOG << "[INFO] Space hashing is off." << std::endl;
         m_pcRayEmbodiedEntityIntersectionMethod = new CRayEmbodiedEntityIntersectionEntitySweep(*this);
      }
      /*
       * Add and initialize all entities in XML
       */
      /* Start from the entities placed manually */
      TConfigurationNodeIterator itArenaItem;
      for(itArenaItem = itArenaItem.begin(&t_tree);
          itArenaItem != itArenaItem.end();
          ++itArenaItem) {
         if(itArenaItem->Value() != "distribute" &&
            itArenaItem->Value() != "box_strip") {
            CEntity* pcEntity = CFactory<CEntity>::New(itArenaItem->Value());
            pcEntity->Init(*itArenaItem);
            CallEntityOperation<CSpaceOperationAddEntity, CSpace, void>(*this, *pcEntity);
         }
      }
      /* Place box strips */
      for(itArenaItem = itArenaItem.begin(&t_tree);
          itArenaItem != itArenaItem.end();
          ++itArenaItem) {
         if(itArenaItem->Value() == "box_strip") {
            AddBoxStrip(*itArenaItem);
         }
      }
      /* Place the entities to distribute automatically */
      for(itArenaItem = itArenaItem.begin(&t_tree);
          itArenaItem != itArenaItem.end();
          ++itArenaItem) {
         if(itArenaItem->Value() == "distribute") {
            Distribute(*itArenaItem);
         }
      }
      if(IsUsingSpaceHash()) {
         /* Initialize the space hash */
         /** @todo make space cell sizing automatic, using the average bb size */
         size_t unBuckets;
         CVector3 cCellSize;
         GetNodeAttributeOrDefault<size_t>(t_tree, "embodied_entity_space_hash_buckets", unBuckets, 100000u);
         GetNodeAttributeOrDefault(t_tree, "embodied_entity_space_hash_cell_size", cCellSize, CVector3(0.2, 0.2, 0.3));
      	 m_pcEmbodiedEntitiesSpaceHash->SetSize(unBuckets);
         m_pcEmbodiedEntitiesSpaceHash->SetCellSize(cCellSize);
         LOG << "[INFO] Embodied entity space hash: " << unBuckets << " buckets, cell size = <" << cCellSize << ">." << std::endl;
         GetNodeAttributeOrDefault<size_t>(t_tree, "led_entity_space_hash_buckets", unBuckets, 100000u);
         GetNodeAttributeOrDefault(t_tree, "led_entity_space_hash_cell_size", cCellSize, CVector3(0.2, 0.2, 0.3));
         m_pcLEDEntitiesSpaceHash->SetSize(unBuckets);
         m_pcLEDEntitiesSpaceHash->SetCellSize(cCellSize);
         LOG << "[INFO] LED entity space hash: " << unBuckets << " buckets, cell size = <" << cCellSize << ">." << std::endl;
         GetNodeAttributeOrDefault<size_t>(t_tree, "rab_equipped_entity_space_hash_buckets", unBuckets, 100000u);
         GetNodeAttributeOrDefault(t_tree, "rab_equipped_entity_space_hash_cell_size", cCellSize, CVector3(1, 1, 1));
         m_pcRABEquippedEntitiesSpaceHash->SetSize(unBuckets);
         m_pcRABEquippedEntitiesSpaceHash->SetCellSize(cCellSize);
         LOG << "[INFO] RAB equipped entity space hash: " << unBuckets << " buckets, cell size = <" << cCellSize << ">." << std::endl;
      }
   }

   /****************************************/
   /****************************************/

   void CSpace::Reset() {
      /* Reset the simulation clock */
      m_unSimulationClock = 0;
      /* Reset the entities */
      for(UInt32 i = 0; i < m_vecEntities.size(); ++i) {
         m_vecEntities[i]->Reset();
      }
      /* Reset the space hash */
      UpdateSpaceData();
   }

   /****************************************/
   /****************************************/

   void CSpace::Destroy() {
      /* Remove all entities */
      while(!m_vecRootEntities.empty()) {
         CallEntityOperation<CSpaceOperationRemoveEntity, CSpace, void>(*this, *m_vecRootEntities.back());
      }
      /* Get rid of the ray-embodied entity intersection method */
      delete m_pcRayEmbodiedEntityIntersectionMethod;
      /* Get rid of the space hashes, if used */
      delete m_pcEmbodiedEntitiesSpaceHash;
      delete m_pcLEDEntitiesSpaceHash;
      delete m_pcRABEquippedEntitiesSpaceHash;
   }

   /****************************************/
   /****************************************/

   void CSpace::GetEntitiesMatching(CEntity::TVector& t_buffer,
                                    const std::string& str_pattern) {
      for(CEntity::TVector::iterator it = m_vecEntities.begin();
          it != m_vecEntities.end(); ++it) {
         if(MatchPattern((*it)->GetId(), str_pattern)) {
            t_buffer.push_back(*it);
         }
      }
   }

   /****************************************/
   /****************************************/

   static void CleanupFoundEntities(TEmbodiedEntitySet& set_found_entities,
                                    const TEmbodiedEntitySet& set_ignored_entities) {
      if(set_ignored_entities.size() == 1) {
         set_found_entities.erase(*set_ignored_entities.begin());
      }
      else {
         for(TEmbodiedEntitySet::const_iterator it = set_ignored_entities.begin();
             it != set_ignored_entities.end(); ++it) {
            set_found_entities.erase(*it);
         }   
      }
   }

   /****************************************/
   /****************************************/

   static bool CalculateClosestCollision(CSpace::SEntityIntersectionItem<CEmbodiedEntity>& s_data,
                                         const CRay3& c_ray,
                                         const TEmbodiedEntitySet& set_found_entities) {
      /* Look for the first entity that actually intersects the ray */
      Real fLowestTSoFar = 0.0f;
      TEmbodiedEntitySet::const_iterator itStart = set_found_entities.begin();
      while(itStart != set_found_entities.end() &&
            ! (*itStart)->CheckIntersectionWithRay(fLowestTSoFar, c_ray)) {
         ++itStart;
      }
      if(itStart != set_found_entities.end()) {
         /* The first entity that intersects the ray has been found */
         CEmbodiedEntity* pcClosestEntitySoFar = *itStart;
         /* Go through the remaining entities */
         Real fTmpT;
         for(TEmbodiedEntitySet::const_iterator it = itStart;
             it != set_found_entities.end(); ++it) {
            if((*it)->CheckIntersectionWithRay(fTmpT, c_ray)) {
               /* Intersection detected, check if it's the closest */
               if(fTmpT < fLowestTSoFar) {
                  fLowestTSoFar = fTmpT;
                  pcClosestEntitySoFar = *it;
               }
            }
         }
         /* Copy the data about the closest intersection */
         s_data.IntersectedEntity = pcClosestEntitySoFar;
         s_data.TOnRay = fLowestTSoFar;
         /* Return true to mean an intersection occurred */
         return true;
      }
      /* No intersection found */
      return false;
   }

   /****************************************/
   /****************************************/

   bool CSpace::GetClosestEmbodiedEntityIntersectedByRay(CSpace::SEntityIntersectionItem<CEmbodiedEntity>& s_data,
                                                         const CRay3& c_ray,
                                                         const TEmbodiedEntitySet& set_ignored_entities) {
      return (*m_pcRayEmbodiedEntityIntersectionMethod)(s_data, c_ray, set_ignored_entities);
   }

   /****************************************/
   /****************************************/

   CSpace::TMapPerType& CSpace::GetEntitiesByType(const std::string& str_type) {
      TMapPerTypePerId::iterator itEntities = m_mapEntitiesPerTypePerId.find(str_type);
      if (itEntities != m_mapEntitiesPerTypePerId.end()){
         return itEntities->second;
      }
      else {
         THROW_ARGOSEXCEPTION("Entity map for type \"" << str_type << "\" not found.");
      }
   }

   /****************************************/
   /****************************************/

   void CSpace::Update() {
      /* Update space-related data */
      UpdateSpaceData();
      /* Update the controllable entities */
      UpdateControllableEntities();
      /* Update the physics engines */
      UpdatePhysics();
      /* Update medium entities */
      UpdateMediumEntities();
      /* @todo remove this, it's only for debugging */
      // LOGERR << "[t=" << m_unSimulationClock << "]" << std::endl;
      // LOGERR << "ALL ENTITIES" << std::endl;
      // for(CEntity::TVector::iterator it = m_vecEntities.begin();
      //     it != m_vecEntities.end();
      //     ++it) {
      //    LOGERR << "   "
      //           << (*it)->GetId()
      //           << std::endl;
      // }
      // LOGERR << "ROOT ENTITIES" << std::endl;
      // for(CEntity::TVector::iterator it = m_vecRootEntities.begin();
      //     it != m_vecRootEntities.end();
      //     ++it) {
      //    LOGERR << "   "
      //           << (*it)->GetId()
      //           << std::endl;
      // }
      // LOGERR << std::endl;
   }

   /****************************************/
   /****************************************/

   void CSpace::AddControllableEntity(CControllableEntity& c_entity) {
      m_vecControllableEntities.push_back(&c_entity);
   }

   /****************************************/
   /****************************************/

   void CSpace::RemoveControllableEntity(CControllableEntity& c_entity) {
      CControllableEntity::TVector::iterator it = find(m_vecControllableEntities.begin(),
                                                       m_vecControllableEntities.end(),
                                                       &c_entity);
      if(it != m_vecControllableEntities.end()) {
         m_vecControllableEntities.erase(it);
      }
   }
      
   /****************************************/
   /****************************************/

   void CSpace::AddMediumEntity(CMediumEntity& c_entity) {
      m_vecMediumEntities.push_back(&c_entity);
   }

   /****************************************/
   /****************************************/

   void CSpace::RemoveMediumEntity(CMediumEntity& c_entity) {
      CMediumEntity::TVector::iterator it = find(m_vecMediumEntities.begin(),
                                                 m_vecMediumEntities.end(),
                                                 &c_entity);
      if(it != m_vecMediumEntities.end()) {
         m_vecMediumEntities.erase(it);
      }
   }

   /****************************************/
   /****************************************/

   void CSpace::AddEntityToPhysicsEngine(CEmbodiedEntity& c_entity) {
      /* Get a reference to the root entity */
      CEntity* pcToAdd = &c_entity;
      while(pcToAdd->HasParent()) {
         pcToAdd = &pcToAdd->GetParent();
      }
      /* Get a reference to the position of the entity */
      const CVector3& cPos = c_entity.GetPosition();
      /* Go through engines and check which ones could house the entity */
      CPhysicsEngine::TVector vecPotentialEngines;
      for(size_t i = 0; i < m_ptPhysicsEngines->size(); ++i) {
         if((*m_ptPhysicsEngines)[i]->IsPointContained(cPos)) {
            vecPotentialEngines.push_back((*m_ptPhysicsEngines)[i]);
         }
      }
      /* If no engine can house the entity, bomb out */
      if(vecPotentialEngines.empty()) {
         THROW_ARGOSEXCEPTION("No physics engine can house entity \"" << pcToAdd->GetId() << "\".");
      }
      /* If the entity is not movable, add the entity to all the matching engines */
      if(! c_entity.IsMovable()) {
         for(size_t i = 0; i < vecPotentialEngines.size(); ++i) {
            vecPotentialEngines[i]->AddEntity(*pcToAdd);
         }
      }
      /* If the entity is movable, only one engine can be associated to the embodied entity */
      else if(vecPotentialEngines.size() == 1) {
         /* Only one engine matches, bingo! */
         vecPotentialEngines[0]->AddEntity(*pcToAdd);
      }
      else {
         /* More than one engine matches, bomb out */
         std::ostringstream ossEngines;
         ossEngines << "\"" << vecPotentialEngines[0]->GetId() << "\"";
         for(size_t i = 1; i < vecPotentialEngines.size(); ++i) {
            ossEngines << ", \"" << vecPotentialEngines[i]->GetId() << "\"";
         }
         THROW_ARGOSEXCEPTION("Multiple engines can house \"" << c_entity.GetId() << "\", but a movable entity and can only be added to a single engine. Conflicting engines: " << ossEngines);
      }
   }
      
   /****************************************/
   /****************************************/

   void CSpace::UpdateSpaceData() {
      /** @todo remove these */
      if(IsUsingSpaceHash()) {
         m_pcEmbodiedEntitiesSpaceHash->Update();
         // LOGERR << "*** EMBODIED ENTITIES ***" << std::endl;
         // m_pcEmbodiedEntitiesSpaceHash->Dump(LOGERR);
         m_pcLEDEntitiesSpaceHash->Update();
         // LOGERR << "*** LED ENTITIES ***" << std::endl;
         // m_pcLEDEntitiesSpaceHash->Dump(LOGERR);
         m_pcRABEquippedEntitiesSpaceHash->Update();
         // LOGERR << "*** RAB ENTITIES ***" << std::endl;
         // m_pcRABEquippedEntitiesSpaceHash->Dump(LOGERR);
      }
   }

   /****************************************/
   /****************************************/

   void CSpace::UpdateMediumEntities() {
      for(size_t i = 0; i < m_vecMediumEntities.size(); ++i) {
         m_vecMediumEntities[i]->Update();
      }
   }

   /****************************************/
   /****************************************/

   class RealNumberGenerator {
   public:
      virtual ~RealNumberGenerator() {}
      virtual CVector3 operator()(bool b_is_retry) = 0;
   };

   class ConstantGenerator : public RealNumberGenerator {
   public:
      ConstantGenerator(const CVector3& c_value) :
         m_cValue(c_value) {}

      inline virtual CVector3 operator()(bool b_is_retry) {
         return m_cValue;
      }
   private:
      CVector3 m_cValue;

   };

   class UniformGenerator : public RealNumberGenerator {
   public:
      UniformGenerator(const CVector3& c_min,
                       const CVector3& c_max) :
         m_cMin(c_min),
         m_cMax(c_max) {}
      inline virtual CVector3 operator()(bool b_is_retry) {
         Real fRandX =
            m_cMax.GetX() > m_cMin.GetX() ?
            CSimulator::GetInstance().GetRNG()->Uniform(CRange<Real>(m_cMin.GetX(), m_cMax.GetX())) :
            m_cMax.GetX();
         Real fRandY =
            m_cMax.GetY() > m_cMin.GetY() ?
            CSimulator::GetInstance().GetRNG()->Uniform(CRange<Real>(m_cMin.GetY(), m_cMax.GetY())) :
            m_cMax.GetY();
         Real fRandZ =
            m_cMax.GetZ() > m_cMin.GetZ() ?
            CSimulator::GetInstance().GetRNG()->Uniform(CRange<Real>(m_cMin.GetZ(), m_cMax.GetZ())) :
            m_cMax.GetZ();
         return CVector3(fRandX, fRandY, fRandZ);
      }
   private:
      CVector3 m_cMin;
      CVector3 m_cMax;
   };

   class GaussianGenerator : public RealNumberGenerator {
   public:
      GaussianGenerator(const CVector3& c_mean,
                        const CVector3& c_std_dev) :
         m_cMean(c_mean),
         m_cStdDev(c_std_dev) {}
      inline virtual CVector3 operator()(bool b_is_retry) {
         return CVector3(CSimulator::GetInstance().GetRNG()->Gaussian(m_cStdDev.GetX(), m_cMean.GetX()),
                         CSimulator::GetInstance().GetRNG()->Gaussian(m_cStdDev.GetY(), m_cMean.GetY()),
                         CSimulator::GetInstance().GetRNG()->Gaussian(m_cStdDev.GetZ(), m_cMean.GetZ()));
      }
   private:
      CVector3 m_cMean;
      CVector3 m_cStdDev;
   };

   class GridGenerator : public RealNumberGenerator {
   public:
      GridGenerator(const CVector3 c_center,
                    const UInt32 un_layout[],
                    const CVector3 c_distances):
         m_cCenter(c_center),
         m_cDistances(c_distances),
         m_unNumEntityPlaced(0) {
         m_unLayout[0] = un_layout[0];
         m_unLayout[1] = un_layout[1];
         m_unLayout[2] = un_layout[2];
         /* Check if layout is sane */
         if( m_unLayout[0] == 0 || m_unLayout[1] == 0 || m_unLayout[2] == 0 ) {
            THROW_ARGOSEXCEPTION("'layout' values (distribute position, method 'grid') must all be different than 0");
         }
      }

      virtual CVector3 operator()(bool b_is_retry) {
         if(b_is_retry) {
            THROW_ARGOSEXCEPTION("Impossible to place entity #" << m_unNumEntityPlaced << " in grid");
         }
         CVector3 cReturn;
         if(m_unNumEntityPlaced < m_unLayout[0] * m_unLayout[1] * m_unLayout[2]) {
            cReturn.SetX(m_cCenter.GetX() + ( m_unLayout[0] - 1 ) * m_cDistances.GetX() * 0.5 - ( m_unNumEntityPlaced  % m_unLayout[0] ) * m_cDistances.GetX());
            cReturn.SetY(m_cCenter.GetY() + ( m_unLayout[1] - 1 ) * m_cDistances.GetY() * 0.5 - ( m_unNumEntityPlaced  / m_unLayout[0] ) % m_unLayout[1] * m_cDistances.GetY());
            cReturn.SetZ(m_cCenter.GetZ() + ( m_unLayout[2] - 1 ) * m_cDistances.GetZ() * 0.5 - ( m_unNumEntityPlaced / ( m_unLayout[0] * m_unLayout[1] ) ) * m_cDistances.GetZ());
            ++m_unNumEntityPlaced;
         }
         else {
            THROW_ARGOSEXCEPTION("Distribute position, method 'grid': trying to place more entities than allowed "
                                 "by the 'layout', check your 'quantity' tag");
         }
         return cReturn;
      }

   private:
      CVector3 m_cCenter;
      UInt32 m_unLayout[3];
      CVector3 m_cDistances;
      UInt32 m_unNumEntityPlaced;
   };

   /****************************************/
   /****************************************/

   RealNumberGenerator* CreateGenerator(TConfigurationNode& t_tree) {
      std::string strMethod;
      GetNodeAttribute(t_tree, "method", strMethod);
      if(strMethod == "uniform") {
         CVector3 cMin, cMax;
         GetNodeAttribute(t_tree, "min", cMin);
         GetNodeAttribute(t_tree, "max", cMax);
         if(! (cMin <= cMax)) {
            THROW_ARGOSEXCEPTION("Uniform generator: the min is not less than or equal to max: " << cMin << " / " << cMax);
         }
         return new UniformGenerator(cMin, cMax);
      }
      else if(strMethod == "gaussian") {
         CVector3 cMean, cStdDev;
         GetNodeAttribute(t_tree, "mean", cMean);
         GetNodeAttribute(t_tree, "std_dev", cStdDev);
         return new GaussianGenerator(cMean, cStdDev);
      }
      else if(strMethod == "constant") {
         CVector3 cValues;
         GetNodeAttribute(t_tree, "values", cValues);
         return new ConstantGenerator(cValues);
      }
      else if(strMethod == "grid") {
         CVector3 cCenter,cDistances;
         GetNodeAttribute(t_tree, "center", cCenter);
         GetNodeAttribute(t_tree, "distances", cDistances);
         UInt32 unLayout[3];
         std::string strLayout;
         GetNodeAttribute(t_tree, "layout", strLayout);
         ParseValues<UInt32> (strLayout, 3, unLayout, ',');
         return new GridGenerator(cCenter, unLayout, cDistances);
      }
      else {
         THROW_ARGOSEXCEPTION("Unknown distribution method \"" << strMethod << "\"");
      }
   }

   /****************************************/
   /****************************************/

   static CEmbodiedEntity* GetEmbodiedEntity(CEntity* pc_entity) {
      /* Is the entity embodied itself? */
      CEmbodiedEntity* pcEmbodiedTest = dynamic_cast<CEmbodiedEntity*>(pc_entity);
      if(pcEmbodiedTest != NULL) {
         return pcEmbodiedTest;
      }
      /* Is the entity composable with an embodied component? */
      CComposableEntity* pcComposableTest = dynamic_cast<CComposableEntity*>(pc_entity);
      if(pcComposableTest != NULL) {
         if(pcComposableTest->HasComponent("body")) {
            return &(pcComposableTest->GetComponent<CEmbodiedEntity>("body"));
         }
      }
      /* No embodied entity found */
      return NULL;
   }

   /****************************************/
   /****************************************/

   static CPositionalEntity* GetPositionalEntity(CEntity* pc_entity) {
      /* Is the entity positional itself? */
      CPositionalEntity* pcPositionalTest = dynamic_cast<CPositionalEntity*>(pc_entity);
      if(pcPositionalTest != NULL) {
         return pcPositionalTest;
      }
      /* Is the entity composable with a positional component? */
      CComposableEntity* pcComposableTest = dynamic_cast<CComposableEntity*>(pc_entity);
      if(pcComposableTest != NULL) {
         if(pcComposableTest->HasComponent("position")) {
            return &(pcComposableTest->GetComponent<CPositionalEntity>("position"));
         }
      }
      /* No positional entity found */
      return NULL;
   }

   /****************************************/
   /****************************************/

   void CSpace::Distribute(TConfigurationNode& t_tree) {
      /* Get the needed nodes */
      TConfigurationNode cPositionNode;
      cPositionNode = GetNode(t_tree, "position");
      TConfigurationNode cOrientationNode;
      cOrientationNode = GetNode(t_tree, "orientation");
      TConfigurationNode cEntityNode;
      cEntityNode = GetNode(t_tree, "entity");
      /* Create the real number generators */
      RealNumberGenerator* pcPositionGenerator = CreateGenerator(cPositionNode);
      RealNumberGenerator* pcOrientationGenerator = CreateGenerator(cOrientationNode);
      /* How many entities? */
      UInt32 unQuantity;
      GetNodeAttribute(cEntityNode, "quantity", unQuantity);
      /* How many trials before failing? */
      UInt32 unMaxTrials;
      GetNodeAttribute(cEntityNode, "max_trials", unMaxTrials);
      /* Get the (optional) entity base numbering */
      UInt64 unBaseNum = 0;
      GetNodeAttributeOrDefault(cEntityNode, "base_num", unBaseNum, unBaseNum);
      /* Get the entity type to add (take only the first, ignore additional if any) */
      TConfigurationNodeIterator itEntity;
      itEntity = itEntity.begin(&cEntityNode);
      if(itEntity == itEntity.end()) {
         THROW_ARGOSEXCEPTION("No entity to distribute specified.");
      }
      /* Get the entity base ID */
      std::string strBaseId;
      GetNodeAttribute(*itEntity, "id", strBaseId);
      /* Add the requested entities */
      for(UInt32 i = 0; i < unQuantity; ++i) {
         /* Copy the entity XML tree */
         TConfigurationNode tEntityTree = *itEntity;
         /* Set progressive ID */
         SetNodeAttribute(tEntityTree, "id", strBaseId + ToString(i+unBaseNum));
         /* Go on until the entity is placed with no collisions or
            the max number of trials has been exceeded */
         UInt32 unTrials = 0;
         bool bDone = false;
         bool bRetry = false;
         CEntity* pcEntity;
         do {
            /* Create entity */
            pcEntity = CFactory<CEntity>::New(tEntityTree.Value());
            /* Set the position */
            SetNodeAttribute(tEntityTree, "position", (*pcPositionGenerator)(bRetry));
            /* Set the orientation */
            SetNodeAttribute(tEntityTree, "orientation", (*pcOrientationGenerator)(bRetry));
            /* Init the entity (this also creates the components, if pcEntity is a composable) */
            pcEntity->Init(tEntityTree);
            /*
             * Now that you have the entity and its components, check whether the entity is positional or embodied
             * or has one such component.
             * In case the entity is positional but not embodied, there's no need to check for collisions
             * In case the entity is embodied, we must check for collisions
             * To check for collisions, we add the entity in the place where it's supposed to be,
             * then we ask the engine if that entity is colliding with something
             * In case of collision, we remove the entity and try a different position/orientation
             */
            /* Check for embodied */
            CEmbodiedEntity* pcEmbodiedEntity = GetEmbodiedEntity(pcEntity);
            if(pcEmbodiedEntity == NULL) {
               /* Check failed, then check for positional */
               CPositionalEntity* pcPositionalEntity = GetPositionalEntity(pcEntity);
               if(pcPositionalEntity == NULL) {
                  THROW_ARGOSEXCEPTION("Cannot distribute entities that are not positional nor embodied, and \"" << tEntityTree.Value() << "\" is neither.");
               }
               else {
                  /* Wherever we want to put the entity, it's OK, add it */
                  CallEntityOperation<CSpaceOperationAddEntity, CSpace, void>(*this, *pcEntity);
               }
            }
            else {
               /* The entity is embodied */
               /* Add it to the space and to the designated physics engine */
               CallEntityOperation<CSpaceOperationAddEntity, CSpace, void>(*this, *pcEntity);
               /* Check if it's colliding with anything else */
               if(pcEmbodiedEntity->IsCollidingWithSomething()) {
                  /* Set retry to true */
                  bRetry = true;
                  /* Get rid of the entity */
                  CallEntityOperation<CSpaceOperationRemoveEntity, CSpace, void>(*this, *pcEntity);
                  /* Increase the trial count */
                  ++unTrials;
                  /* Too many trials? */
                  if(unTrials > unMaxTrials) {
                     /* Yes, bomb out */
                     THROW_ARGOSEXCEPTION("Exceeded max trials when trying to distribute objects of type " <<
                                          tEntityTree.Value() << " with base id \"" <<
                                          strBaseId << "\". I managed to place only " << i << " objects.");
                  }
                  /* Retry with a new position */
               }
               else {
                  /* No collision, we're done with this entity */
                  bDone = true;
               }
            }
         }
         while(!bDone);
      }
      /* Delete the generators, now unneeded */
      delete pcPositionGenerator;
      delete pcOrientationGenerator;
   }

   /****************************************/
   /****************************************/

   void CSpace::AddBoxStrip(TConfigurationNode& t_tree){

      // this function can convert a inkscape path into a box strip
      // in inkscape: preference -> svg output -> deny relative coordinates

	    // read parameters
      std::string strBaseId;
      GetNodeAttribute(t_tree, "id", strBaseId);
      std::string strCoordinates;
      GetNodeAttribute(t_tree, "coordinates", strCoordinates);
      Real fScale;
      GetNodeAttributeOrDefault<Real>(t_tree, "scale", fScale, 1.0f);
      Real fBoxWidth;
      GetNodeAttribute(t_tree, "boxwidth", fBoxWidth);
      Real fBoxHeight;
      GetNodeAttribute(t_tree, "boxheight", fBoxHeight);
      CVector2 cTranslate;
      GetNodeAttributeOrDefault(t_tree, "translate", cTranslate, CVector2(0,0));
      bool bClose;
      GetNodeAttributeOrDefault(t_tree, "close", bClose, false);
      bool bRoundEdges;
      GetNodeAttributeOrDefault(t_tree, "roundedge", bRoundEdges, true);

      std::vector<std::string> vecCoordinates;
      Tokenize(strCoordinates, vecCoordinates,", ");

      CVector2 cA, cB, cC;

      for (UInt32 i = 0; i < vecCoordinates.size() - (bClose ? 0 : 2); i += 2) {

         // read the next three points in strip
         cA.Set(FromString<Real>(vecCoordinates[i]), FromString<Real>(vecCoordinates[i+1]));
         cA = cA * fScale + cTranslate;
         cB.Set(FromString<Real>(vecCoordinates[(i+2) % vecCoordinates.size()]), FromString<Real>(vecCoordinates[(i+3) % vecCoordinates.size()]));
         cB = cB * fScale + cTranslate;
         cC.Set(FromString<Real>(vecCoordinates[(i+4) % vecCoordinates.size()]), FromString<Real>(vecCoordinates[(i+5) % vecCoordinates.size()]));
         cC = cC * fScale + cTranslate;

         CEntity* pcEntity = CFactory<CEntity>::New("box");

         CVector3 cOrientationAngles(ToDegrees((cB-cA).Angle()).GetValue(), 0., 0.);
         /* Build the XML tree */
         TConfigurationNode tRootNode("box");
         SetNodeAttribute(tRootNode, "id", strBaseId+ "_" + ToString(i));
         SetNodeAttribute(tRootNode, "position", CVector3(((cA+cB)/2.).GetX(), ((cA+cB)/2.).GetY(), 0.));
         SetNodeAttribute(tRootNode, "orientation", cOrientationAngles);
         SetNodeAttribute(tRootNode, "size", CVector3((cA-cB).Length(), fBoxWidth, fBoxHeight));
         SetNodeAttribute(tRootNode, "movable", false);
         SetNodeAttribute(tRootNode, "mass", 1);
         pcEntity->Init(tRootNode);
         AddEntity(*pcEntity);

         if (bRoundEdges && (bClose || i < vecCoordinates.size()-4)){

            pcEntity = CFactory<CEntity>::New("box");

            cOrientationAngles.SetX(ToDegrees((cB-cA).Angle()+(cB-cC).Angle()).GetValue() / 2.);
            /* Build the XML tree */
            SetNodeAttribute(tRootNode, "id", strBaseId+ "_" + ToString(i + vecCoordinates.size()));
            SetNodeAttribute(tRootNode, "orientation", cOrientationAngles);

            CVector2 cPerp;
            cPerp.SetFromAngleAndLength((cB-cA).Angle() + CRadians::PI_OVER_TWO, fBoxWidth/2);
            cPerp += cB;

            CVector2 cPerp2;
            cPerp2.SetFromAngleAndLength((cB-cC).Angle() - CRadians::PI_OVER_TWO, fBoxWidth/2);
            cPerp2 += cB;

            SetNodeAttribute(tRootNode, "position", CVector3(cB.GetX(), cB.GetY(), 0.));
            SetNodeAttribute(tRootNode, "size", CVector3(sqrt(fBoxWidth*fBoxWidth - Distance(cPerp,cPerp2)*Distance(cPerp,cPerp2)),Distance(cPerp,cPerp2), fBoxHeight));

            pcEntity->Init(tRootNode);
            AddEntity(*pcEntity);
         }
      }
   }

   /****************************************/
   /****************************************/

   bool CSpace::GetClosestEmbodiedEntityIntersectedByRaySpaceHash(SEntityIntersectionItem<CEmbodiedEntity>& s_data,
                                                                  const CRay3& c_ray,
                                                                  const TEmbodiedEntitySet& set_ignored_entities) {
      /* Buffer for embodied entities found along the ray */
      TEmbodiedEntitySet tFoundEntities;
      /* Transform ray start and end position into cell coordinates */
      SInt32 nI1, nJ1, nK1, nI2, nJ2, nK2;
      m_pcEmbodiedEntitiesSpaceHash->SpaceToHashTable(nI1, nJ1, nK1, c_ray.GetStart());
      m_pcEmbodiedEntitiesSpaceHash->SpaceToHashTable(nI2, nJ2, nK2, c_ray.GetEnd());
      /* Go through cells one by one, from start to end.
         Stop as soon as an entity is found.
         If the loop is completed, it means no entities were found -> no intersection.
      */
      /* Calculate deltas for later use */
      SInt32 nDI(Abs(nI2 - nI1));
      SInt32 nDJ(Abs(nJ2 - nJ1));
      SInt32 nDK(Abs(nK2 - nK1));
      /* Calculate the increment for each direction */
      SInt32 nSI(nI2 >= nI1 ? 1 : -1);
      SInt32 nSJ(nJ2 >= nJ1 ? 1 : -1);
      SInt32 nSK(nK2 >= nK1 ? 1 : -1);
      /* Set the starting cell */
      SInt32 nI(nI1), nJ(nJ1), nK(nK1);
      if(nDI >= nDJ && nDI >= nDK) {
         /* I is the driving axis */
         /* Calculate error used to know when to move on other axes */
         SInt32 nEJ(3 * nDJ - nDI);
         SInt32 nEK(3 * nDK - nDI);
         m_pcEmbodiedEntitiesSpaceHash->CheckCell(nI, nJ, nK, tFoundEntities);
         /* Check if the found entities have an intersection.
            If so, we can stop. Otherwise, we continue. */
         if(! set_ignored_entities.empty()) {
            CleanupFoundEntities(tFoundEntities, set_ignored_entities);
         }
         if(! tFoundEntities.empty() &&
            CalculateClosestCollision(s_data,
                                      c_ray,
                                      tFoundEntities)) {
            return true;
         }
         /* Cycle through cells */
         for(SInt32 nCell = nDI; nCell > 0; --nCell) {
            /* Clean up the found entities */
            tFoundEntities.clear();
            /* Advance on driving axis */
            nI += nSI;
            m_pcEmbodiedEntitiesSpaceHash->CheckCell(nI, nJ, nK, tFoundEntities);
            /* Advance on other axes, if necessary */
            if(nEJ > 0 && nEK > 0) {
               /* Advance on both the other axes */
               if(nEJ * nDK > nEK * nDJ) {
                  nJ += nSJ;
                  m_pcEmbodiedEntitiesSpaceHash->CheckCell(nI, nJ, nK, tFoundEntities);
                  nK += nSK;
                  m_pcEmbodiedEntitiesSpaceHash->CheckCell(nI, nJ, nK, tFoundEntities);
               }
               else {
                  nK += nSK;
                  m_pcEmbodiedEntitiesSpaceHash->CheckCell(nI, nJ, nK, tFoundEntities);
                  nJ += nSJ;
                  m_pcEmbodiedEntitiesSpaceHash->CheckCell(nI, nJ, nK, tFoundEntities);
               }
               nEJ += 2 * (nDJ - nDI);
               nEK += 2 * (nDK - nDI);
            }
            else if(nEJ > 0) {
               /* Advance only on J */
               nJ += nSJ;
               m_pcEmbodiedEntitiesSpaceHash->CheckCell(nI, nJ, nK, tFoundEntities);
               nEJ += 2 * (nDJ - nDI);
               nEK += 2 * nDK;
            }
            else {
               nEJ += 2 * nDJ;
               if(nEK > 0) {
                  /* Advance only on K */
                  nK += nSK;
                  m_pcEmbodiedEntitiesSpaceHash->CheckCell(nI, nJ, nK, tFoundEntities);
                  nEK += 2 * (nDK - nDI);
               }
               else {
                  nEK += 2 * nDK;
               }
            }
            /* Check if the found entities have an intersection.
               If so, we can stop. Otherwise, we continue. */
            if(! set_ignored_entities.empty()) {
               CleanupFoundEntities(tFoundEntities, set_ignored_entities);
            }
            if(! tFoundEntities.empty() &&
               CalculateClosestCollision(s_data,
                                         c_ray,
                                         tFoundEntities)) {
               return true;
            }
         }
      }
      else if(nDJ >= nDI && nDJ >= nDK) {
         /* J is the driving axis */
         /* Calculate error used to know when to move on other axes */
         SInt32 nEI(3 * nDI - nDJ);
         SInt32 nEK(3 * nDK - nDJ);
         m_pcEmbodiedEntitiesSpaceHash->CheckCell(nI, nJ, nK, tFoundEntities);
         /* Check if the found entities have an intersection.
            If so, we can stop. Otherwise, we continue. */
         if(! set_ignored_entities.empty()) {
            CleanupFoundEntities(tFoundEntities, set_ignored_entities);
         }
         if(! tFoundEntities.empty() &&
            CalculateClosestCollision(s_data,
                                      c_ray,
                                      tFoundEntities)) {
            return true;
         }
         /* Cycle through cells */
         for(SInt32 nCell = nDJ; nCell > 0; --nCell) {
            /* Clean up the found entities */
            tFoundEntities.clear();
            /* Advance on driving axis */
            nJ += nSJ;
            m_pcEmbodiedEntitiesSpaceHash->CheckCell(nI, nJ, nK, tFoundEntities);
            /* Advance on other axes, if necessary */
            if(nEI > 0 && nEK > 0) {
               /* Advance on both the other axes */
               if(nEI * nDK > nEK * nDI) {
                  nI += nSI;
                  m_pcEmbodiedEntitiesSpaceHash->CheckCell(nI, nJ, nK, tFoundEntities);
                  nK += nSK;
                  m_pcEmbodiedEntitiesSpaceHash->CheckCell(nI, nJ, nK, tFoundEntities);
               }
               else {
                  nK += nSK;
                  m_pcEmbodiedEntitiesSpaceHash->CheckCell(nI, nJ, nK, tFoundEntities);
                  nI += nSI;
                  m_pcEmbodiedEntitiesSpaceHash->CheckCell(nI, nJ, nK, tFoundEntities);
               }
               nEI += 2 * (nDI - nDJ);
               nEK += 2 * (nDK - nDJ);
            }
            else if(nEI > 0) {
               /* Advance only on I */
               nI += nSI;
               m_pcEmbodiedEntitiesSpaceHash->CheckCell(nI, nJ, nK, tFoundEntities);
               nEI += 2 * (nDI - nDJ);
               nEK += 2 * nDK;
            }
            else {
               nEI += 2 * nDI;
               if(nEK > 0) {
                  /* Advance only on K */
                  nK += nSK;
                  m_pcEmbodiedEntitiesSpaceHash->CheckCell(nI, nJ, nK, tFoundEntities);
                  nEK += 2 * (nDK - nDJ);
               }
               else {
                  nEK += 2 * nDK;
               }
            }
            /* Check if the found entities have an intersection.
               If so, we can stop. Otherwise, we continue. */
            if(! set_ignored_entities.empty()) {
               CleanupFoundEntities(tFoundEntities, set_ignored_entities);
            }
            if(! tFoundEntities.empty() &&
               CalculateClosestCollision(s_data,
                                         c_ray,
                                         tFoundEntities)) {
               return true;
            }
         }
      }
      else {
         /* K is the driving axis */
         /* Calculate error used to know when to move on other axes */
         SInt32 nEI(3 * nDI - nDK);
         SInt32 nEJ(3 * nDJ - nDK);
         m_pcEmbodiedEntitiesSpaceHash->CheckCell(nI, nJ, nK, tFoundEntities);
         /* Check if the found entities have an intersection.
            If so, we can stop. Otherwise, we continue. */
         if(! set_ignored_entities.empty()) {
            CleanupFoundEntities(tFoundEntities, set_ignored_entities);
         }
         if(! tFoundEntities.empty() &&
            CalculateClosestCollision(s_data,
                                      c_ray,
                                      tFoundEntities)) {
            return true;
         }
         /* Cycle through cells */
         for(SInt32 nCell = nDK; nCell > 0; --nCell) {
            /* Clean up the found entities */
            tFoundEntities.clear();
            /* Advance on driving axis */
            nK += nSK;
            m_pcEmbodiedEntitiesSpaceHash->CheckCell(nI, nJ, nK, tFoundEntities);
            /* Advance on other axes, if necessary */
            if(nEI > 0 && nEJ > 0) {
               /* Advance on both the other axes */
               if(nEI * nDJ > nEJ * nDI) {
                  nI += nSI;
                  m_pcEmbodiedEntitiesSpaceHash->CheckCell(nI, nJ, nK, tFoundEntities);
                  nJ += nSJ;
                  m_pcEmbodiedEntitiesSpaceHash->CheckCell(nI, nJ, nK, tFoundEntities);
               }
               else {
                  nJ += nSJ;
                  m_pcEmbodiedEntitiesSpaceHash->CheckCell(nI, nJ, nK, tFoundEntities);
                  nI += nSI;
                  m_pcEmbodiedEntitiesSpaceHash->CheckCell(nI, nJ, nK, tFoundEntities);
               }
               nEI += 2 * (nDI - nDK);
               nEJ += 2 * (nDJ - nDK);
            }
            else if(nEI > 0) {
               /* Advance only on I */
               nI += nSI;
               m_pcEmbodiedEntitiesSpaceHash->CheckCell(nI, nJ, nK, tFoundEntities);
               nEI += 2 * (nDI - nDK);
               nEJ += 2 * nDJ;
            }
            else {
               nEI += 2 * nDI;
               if(nEJ > 0) {
                  /* Advance only on J */
                  nJ += nSJ;
                  m_pcEmbodiedEntitiesSpaceHash->CheckCell(nI, nJ, nK, tFoundEntities);
                  nEJ += 2 * (nDJ - nDK);
               }
               else {
                  nEJ += 2 * nDJ;
               }
            }
            /* Check if the found entities have an intersection.
               If so, we can stop. Otherwise, we continue. */
            if(! set_ignored_entities.empty()) {
               CleanupFoundEntities(tFoundEntities, set_ignored_entities);
            }
            if(! tFoundEntities.empty() &&
               CalculateClosestCollision(s_data,
                                         c_ray,
                                         tFoundEntities)) {
               return true;
            }
         }
      }
      /* End of the loop reached and no entities found. */
      return false;
   }

   /****************************************/
   /****************************************/

   bool CSpace::GetClosestEmbodiedEntityIntersectedByRayEntitySweep(SEntityIntersectionItem<CEmbodiedEntity>& s_data,
                                                                    const CRay3& c_ray,
                                                                    const TEmbodiedEntitySet& set_ignored_entities) {
      return false;
   }

   /****************************************/
   /****************************************/

}
