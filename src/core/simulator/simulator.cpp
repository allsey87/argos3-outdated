/**
 * @file <argos3/core/simulator/simulator.cpp>
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 */

#include "simulator.h"

#include <iostream>
#include <string>
#include <sys/time.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/utility/profiler/profiler.h>
#include <argos3/core/utility/string_utilities.h>
#include <argos3/core/utility/plugins/dynamic_loading.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/simulator/space/space_no_threads.h>
#include <argos3/core/simulator/space/space_multi_thread_balance_quantity.h>
#include <argos3/core/simulator/space/space_multi_thread_balance_length.h>
#include <argos3/core/simulator/visualization/visualization.h>
#include <argos3/core/simulator/physics_engine/physics_engine.h>
#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/physics_engine/physics_engine_entity.h>

namespace argos {

   /****************************************/
   /****************************************/

   CSimulator::CSimulator() :
      m_pcVisualization(NULL),
      m_pcSpace(NULL),
      m_pcLoopFunctions(NULL),
      m_unMaxSimulationClock(0),
      m_bWasRandomSeedSet(false),
      m_unThreads(0),
      m_pcProfiler(NULL),
      m_bHumanReadableProfile(true) {}

   /****************************************/
   /****************************************/

   CSimulator::~CSimulator() {
      if(IsProfiling()) {
         delete m_pcProfiler;
      }
      /* Delete the visualization */
      if(m_pcVisualization != NULL) delete m_pcVisualization;
      /* Delete all the physics engines */
      for(CPhysicsEngine::TMap::iterator it = m_mapPhysicsEngines.begin(); it
             != m_mapPhysicsEngines.end(); ++it)
         delete it->second;
      m_mapPhysicsEngines.clear();
      m_vecPhysicsEngines.clear();
      /* Delete the space and the dynamic linking manager */
      if(m_pcSpace != NULL) {
         delete m_pcSpace;
      }
   }

   /****************************************/
   /****************************************/

   CSimulator& CSimulator::GetInstance() {
      static std::auto_ptr<CSimulator> pcSimulatorInstance(new CSimulator());
      return *(pcSimulatorInstance.get());
   }

   /****************************************/
   /****************************************/

   CPhysicsEngine& CSimulator::GetPhysicsEngine(const std::string& str_id) const {
      CPhysicsEngine::TMap::const_iterator it = m_mapPhysicsEngines.find(str_id);
      ARGOS_ASSERT(it != m_mapPhysicsEngines.end(), "Physics engine \"" << str_id << "\" not found.")
         return *(it->second);
   }

   /****************************************/
   /****************************************/

   TConfigurationNode& CSimulator::GetConfigForController(const std::string& str_id) {
      TControllerConfigurationMap::iterator it = m_mapControllerConfig.find(str_id);
      if(it == m_mapControllerConfig.end()) {
         THROW_ARGOSEXCEPTION("Can't find XML configuration for controller id \"" << str_id << "\"");
      }
      return *(it->second);
   }

   /****************************************/
   /****************************************/

   void CSimulator::LoadExperiment() {
      /* Build configuration tree */
      m_tConfiguration.LoadFile(m_strExperimentConfigFileName);
      m_tConfigurationRoot = *m_tConfiguration.FirstChildElement();
      /* Init the experiment */
      Init();
      LOG.Flush();
      LOGERR.Flush();
   }

   /****************************************/
   /****************************************/

   void CSimulator::Init() {
      /* General configuration */
      InitFramework(GetNode(m_tConfigurationRoot, "framework"));
      /* Initialize controllers */
      InitControllers(GetNode(m_tConfigurationRoot, "controllers"));
      /* Create loop functions */
      bool bUserDefLoopFunctions;
      if(NodeExists(m_tConfigurationRoot, "loop_functions")) {
         /* User specified a loop_functions section in the XML */
         InitLoopFunctions(GetNode(m_tConfigurationRoot, "loop_functions"));
         bUserDefLoopFunctions = true;
      }
      else {
         /* No loop_functions in the XML */
         m_pcLoopFunctions = new CLoopFunctions;
         bUserDefLoopFunctions = false;
      }
      /* Physics engines */
      InitPhysics(GetNode(m_tConfigurationRoot, "physics_engines"));
      /* Space */
      InitSpace(GetNode(m_tConfigurationRoot, "arena"));
      /* Call user init function */
      if(bUserDefLoopFunctions) {
         m_pcLoopFunctions->Init(GetNode(m_tConfigurationRoot, "loop_functions"));
      }
      /* Initialise visualization */
      TConfigurationNodeIterator itVisualization;
      if(NodeExists(m_tConfigurationRoot, "visualization") &&
         ((itVisualization = itVisualization.begin(&GetNode(m_tConfigurationRoot, "visualization"))) != itVisualization.end())) {
         InitVisualization(GetNode(m_tConfigurationRoot, "visualization"));
      }
      else {
         LOGERR << "[WARNING] No visualization selected." << std::endl;
         m_pcVisualization = new CVisualization();
      }
      /* Start profiling, if needed */
      if(IsProfiling()) {
         m_pcProfiler->Start();
      }
   }

   /****************************************/
   /****************************************/

   void CSimulator::Reset() {
      /* if random seed is 0 or is not specified, init with the current timeval */
      if(m_bWasRandomSeedSet) {
         CRandom::SetSeedOf("argos", m_unRandomSeed);
      }
      else {
         /* Prepare the default value based on the current clock time */
         struct timeval sTimeValue;
         ::gettimeofday(&sTimeValue, NULL);
         UInt32 unSeed = static_cast<UInt32>(sTimeValue.tv_usec);
         CRandom::SetSeedOf("argos", unSeed);
         m_unRandomSeed = unSeed;
         LOG << "[INFO] Using random seed = " << m_unRandomSeed << std::endl;
      }
      CRandom::GetCategory("argos").ResetRNGs();

      /* Reset the space */
      m_pcSpace->Reset();

      /* Reset the physics engines */
      for(CPhysicsEngine::TMap::iterator it = m_mapPhysicsEngines.begin();
          it != m_mapPhysicsEngines.end(); ++it) {
         it->second->Reset();
      }

      /* Reset the loop functions */
      m_pcLoopFunctions->Reset();

      LOG.Flush();
      LOGERR.Flush();
   }

   /****************************************/
   /****************************************/

   void CSimulator::Destroy() {
      /* Call user destroy function */
      if (m_pcLoopFunctions != NULL) {
         m_pcLoopFunctions->Destroy();
         delete m_pcLoopFunctions;
         m_pcLoopFunctions = NULL;
      }

      /* Destroy the visualization */
      if(m_pcVisualization != NULL) {
         m_pcVisualization->Destroy();
      }

      /* Close physics engines */
      for(CPhysicsEngine::TMap::iterator it = m_mapPhysicsEngines.begin();
          it != m_mapPhysicsEngines.end(); ++it) {
         it->second->Destroy();
         delete it->second;
      }
      m_mapPhysicsEngines.clear();
      m_vecPhysicsEngines.clear();

      /* Destroy simulated space */
      if(m_pcSpace != NULL) {
         m_pcSpace->Destroy();
      }

      /* Get rid of ARGoS category */
      if(CRandom::ExistsCategory("argos")) {
         CRandom::RemoveCategory("argos");
      }

      /* Free up factory data */
      CFactory<CPhysicsEngine>::Destroy();
      CFactory<CVisualization>::Destroy();
      CFactory<CSimulatedActuator>::Destroy();
      CFactory<CSimulatedSensor>::Destroy();
      CFactory<CCI_Controller>::Destroy();
      CFactory<CEntity>::Destroy();

      /* Stop profiling and flush the data */
      if(IsProfiling()) {
         m_pcProfiler->Stop();
         m_pcProfiler->Flush(m_bHumanReadableProfile);
      }

      LOG.Flush();
      LOGERR.Flush();
   }

   /****************************************/
   /****************************************/

   void CSimulator::Execute() {
      m_pcVisualization->Execute();
   }

   /****************************************/
   /****************************************/

   void CSimulator::UpdateSpace() {
      /* Increase the simulation clock */
      m_pcSpace->IncreaseSimulationClock();
      /* Call loop functions */
      m_pcLoopFunctions->PreStep();
      LOG.Flush();
      LOGERR.Flush();
      /* Update the space */
      m_pcSpace->Update();
      LOG.Flush();
      LOGERR.Flush();
      /* Call loop functions */
      m_pcLoopFunctions->PostStep();
      LOG.Flush();
      LOGERR.Flush();
   }

   /****************************************/
   /****************************************/

   bool CSimulator::IsExperimentFinished() const {
      /*
        The experiment is considered finished when the simulation clock exceeds
        the maximum value set in the XML, or when one of the visualisations asks
        to terminate.
      */

      /* Check simulation clock */
      if (m_unMaxSimulationClock > 0 &&
          m_pcSpace->GetSimulationClock() >= m_unMaxSimulationClock) {
         return true;
      }

      /* Call loop function */
      return m_pcLoopFunctions->IsExperimentFinished();
   }

   /****************************************/
   /****************************************/

   void CSimulator::InitFramework(TConfigurationNode& t_tree) {
      try {
         /* Parse the 'system' node */
         if(NodeExists(t_tree, "system")) {
            TConfigurationNode tSystem;
            tSystem = GetNode(t_tree, "system");
            GetNodeAttributeOrDefault(tSystem, "threads", m_unThreads, m_unThreads);
            if(m_unThreads == 0) {
               LOG << "[INFO] Not using threads" << std::endl;
               m_pcSpace = new CSpaceNoThreads;
            }
            else {
               LOG << "[INFO] Using " << m_unThreads << " parallel threads" << std::endl;
               std::string strThreadingMethod = "balance_quantity";
               GetNodeAttributeOrDefault(tSystem, "method", strThreadingMethod, strThreadingMethod);
               if(strThreadingMethod == "balance_quantity") {
                  LOG << "[INFO]   Method \"balance_quantity\" chosen: threads will be assigned the same"
                      << std::endl
                      << "[INFO]   number of tasks, independently of the task length."
                      << std::endl;
                  m_pcSpace = new CSpaceMultiThreadBalanceQuantity;
               }
               else if(strThreadingMethod == "balance_length") {
                  LOG << "[INFO]   Method \"balance_quantity\" chosen: threads will be assigned different"
                      << std::endl
                      << "[INFO]   numbers of tasks, depending on the task length."
                      << std::endl;
                  m_pcSpace = new CSpaceMultiThreadBalanceLength;
               }
               else {
                  THROW_ARGOSEXCEPTION("Error parsing the <system> tag. Unknown threading method \"" << strThreadingMethod << "\". Available methods: \"balance_quantity\" and \"balance_length\".");
               }
            }
         }
         else {
            LOG << "[INFO] Not using threads" << std::endl;
            m_pcSpace = new CSpaceNoThreads;
         }

         /* Get 'experiment' node */
         TConfigurationNode tExperiment;
         tExperiment = GetNode(t_tree, "experiment");

         /* Parse random seed */
         /* Buffer to hold the random seed */
         GetNodeAttributeOrDefault(tExperiment,
                                   "random_seed",
                                   m_unRandomSeed,
                                   static_cast<UInt32>(0));

         /* if random seed is 0 or is not specified, init with the current timeval */
         if(m_unRandomSeed != 0) {
            CRandom::CreateCategory("argos", m_unRandomSeed);
            LOG << "[INFO] Using random seed = " << m_unRandomSeed << std::endl;
            m_bWasRandomSeedSet = true;
         }
         else {
            /* Prepare the default value based on the current clock time */
            m_bWasRandomSeedSet = false;
            struct timeval sTimeValue;
            ::gettimeofday(&sTimeValue, NULL);
            UInt32 unSeed = static_cast<UInt32>(sTimeValue.tv_usec);
            m_unRandomSeed = unSeed;
            CRandom::CreateCategory("argos", unSeed);
            LOG << "[INFO] Using random seed = " << unSeed << std::endl;
         }
         m_pcRNG = CRandom::CreateRNG("argos");

         /* Set the simulation clock tick length */
         UInt32 unTicksPerSec;
         GetNodeAttribute(tExperiment,
                          "ticks_per_second",
                          unTicksPerSec);
         CPhysicsEngine::SetSimulationClockTick(1.0 / static_cast<Real>(unTicksPerSec));

         /* Set the maximum simulation duration (in seconds) */
         Real fExpLength;
         GetNodeAttributeOrDefault<Real>(tExperiment,
                                         "length",
                                         fExpLength,
                                         0.0f);
         m_unMaxSimulationClock = fExpLength * unTicksPerSec;
         LOG << "[INFO] Total experiment length in clock ticks = "
             << (m_unMaxSimulationClock ? ToString(m_unMaxSimulationClock) : "unlimited")
             << std::endl;

         /* Get the profiling tag, if present */
         if(NodeExists(t_tree, "profiling")) {
            TConfigurationNode& tProfiling = GetNode(t_tree, "profiling");
            std::string strFile;
            GetNodeAttribute(tProfiling, "file", strFile);
            std::string strFormat;
            GetNodeAttribute(tProfiling, "format", strFormat);
            if(strFormat == "human_readable") {
               m_bHumanReadableProfile = true;
            }
            else if(strFormat == "table") {
               m_bHumanReadableProfile = false;
            }
            else {
               THROW_ARGOSEXCEPTION("Unrecognized profile format \"" << strFormat << "\". Accepted values are \"human_readable\" and \"table\".");
            }
            bool bTrunc = true;
            GetNodeAttributeOrDefault(tProfiling, "truncate_file", bTrunc, bTrunc);
            m_pcProfiler = new CProfiler(strFile, bTrunc);
         }
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Failed to initialize the simulator. Parse error inside the <framework> tag.", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CSimulator::InitLoopFunctions(TConfigurationNode& t_tree) {
      try {
         std::string strLibrary, strLabel;
         GetNodeAttribute(t_tree, "library", strLibrary);
         GetNodeAttribute(t_tree, "label", strLabel);
         CDynamicLoading::LoadLibrary(strLibrary);
         m_pcLoopFunctions = CFactory<CLoopFunctions>::New(strLabel);
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Error initializing loop functions", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CSimulator::InitControllers(TConfigurationNode& t_tree) {
      /*
       * Go through controllers, loading the library of each of them
       * and storing type, id and XML tree of each of them for later use
       */
      if(! t_tree.NoChildren()) {
         try {
            std::string strLibrary;
            std::string strId;
            TConfigurationNodeIterator it;
            for(it = it.begin(&t_tree);
                it != it.end(); ++it) {
               /* Get controller id */
               try {
                  GetNodeAttribute(*it, "id", strId);
               }
               catch(CARGoSException& ex) {
                  std::string strValue;
                  it->GetValue(&strValue);
                  THROW_ARGOSEXCEPTION_NESTED("Controller type \"" << strValue << "\" has no assigned id.", ex);
               }
               /* Bomb out if id is already in map */
               if(m_mapControllerConfig.find(strId) != m_mapControllerConfig.end()) {
                  THROW_ARGOSEXCEPTION("Controller id \"" << strId << "\" duplicated");
               }
               /* Optionally, process "library" attribute if present */
               if(NodeAttributeExists(*it, "library")) {
                  /* Get library name */
                  GetNodeAttribute(*it, "library", strLibrary);
                  /* Load library */
                  CDynamicLoading::LoadLibrary(strLibrary);
               }
               /* Store XML info in map by id */
               m_mapControllerConfig.insert(std::pair<std::string, TConfigurationNode*>(strId, &(*it)));
            }
         }
         catch(CARGoSException& ex) {
            THROW_ARGOSEXCEPTION_NESTED("Error initializing controllers", ex);
         }
      }
   }

   /****************************************/
   /****************************************/

   void CSimulator::InitSpace(TConfigurationNode& t_tree) {
      try {
         m_pcSpace->Init(t_tree);
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Failed to initialize the space.", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CSimulator::InitPhysics(TConfigurationNode& t_tree) {
      try {
         /* Cycle through the physics engines */
         TConfigurationNodeIterator itEngines;
         for(itEngines = itEngines.begin(&t_tree);
             itEngines != itEngines.end();
             ++itEngines) {
            /* Create the physics engine */
            CPhysicsEngine* pcEngine = CFactory<CPhysicsEngine>::New(itEngines->Value());
            try {
               /* Initialize the engine */
               pcEngine->Init(*itEngines);
               /* Check that an engine with that ID does not exist yet */
               if(m_mapPhysicsEngines.find(pcEngine->GetId()) == m_mapPhysicsEngines.end()) {
                  /* Add it to the lists */
                  m_mapPhysicsEngines[pcEngine->GetId()] = pcEngine;
                  m_vecPhysicsEngines.push_back(pcEngine);
               }
               else {
                  /* Duplicate id -> error */
                  THROW_ARGOSEXCEPTION("A physics engine with id \"" << pcEngine->GetId() << "\" exists already. The ids must be unique!");
               }
            }
            catch(CARGoSException& ex) {
               /* Error while executing engine init, destroy what done to prevent memory leaks */
               pcEngine->Destroy();
               delete pcEngine;
               THROW_ARGOSEXCEPTION_NESTED("Error initializing physics engine type \"" << itEngines->Value() << "\"", ex);
            }
         }
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Failed to initialize the physics engines. Parse error in the <physics_engines> subtree.", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CSimulator::InitVisualization(TConfigurationNode& t_tree) {
      try {
         /* Consider only the first visualization */
         TConfigurationNodeIterator itVisualization;
         itVisualization = itVisualization.begin(&t_tree);
         /* Create the visualization */
         m_pcVisualization = CFactory<CVisualization>::New(itVisualization->Value());
         /* Initialize the visualization */
         m_pcVisualization->Init(*itVisualization);
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Failed to initialize the visualization. Parse error in the <visualization> subtree.", ex);
      }
   }

   /****************************************/
   /****************************************/

}
