/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_engine.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "dynamics3d_engine.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/entity/embodied_entity.h>

//#include <cstring>
//#include <algorithm>

namespace argos {

   /****************************************/
   /****************************************/

   CDynamics3DEngine::CDynamics3DEngine() :
      m_pcBroadphaseInterface(NULL),
      m_pcCollisionConfiguration(NULL),
      m_pcCollisionDispatcher(NULL),
      m_pcSolver(NULL),
      m_pcWorld(NULL),
      m_pcGhostPairCallback(NULL),
      m_pcGroundCollisionShape(NULL),
      m_pcGroundMotionState(NULL),
      m_pcGroundRigidBody(NULL) {}

   /****************************************/
   /****************************************/

   void CDynamics3DEngine::Init(TConfigurationNode& t_tree) {
      /* Init parent */
      CPhysicsEngine::Init(t_tree);
      
      /* Parse the XML */
      std::string strBroadphaseAlgorithm("dynamic_aabb_tree");
      GetNodeAttributeOrDefault(t_tree, "broadphase_algorithm", strBroadphaseAlgorithm, strBroadphaseAlgorithm);
      
      if(strBroadphaseAlgorithm == "dynamic_aabb_tree") {
         m_pcBroadphaseInterface = new btDbvtBroadphase();
      }
      else {
         THROW_ARGOSEXCEPTION("The broadphase algorithm " << strBroadphaseAlgorithm << " is not implemented");
      }
      
      /* Select the default configurations, dispatches and solvers */
      m_pcCollisionConfiguration = new btDefaultCollisionConfiguration();
      m_pcCollisionDispatcher = new btCollisionDispatcher(m_pcCollisionConfiguration);
      m_pcSolver = new btSequentialImpulseConstraintSolver;

      /* Create the physics world */
      m_pcWorld = new btDiscreteDynamicsWorld(m_pcCollisionDispatcher,
                                              m_pcBroadphaseInterface,
                                              m_pcSolver,
                                              m_pcCollisionConfiguration);
       
      /* Set the gravity in the world */
      m_pcWorld->setGravity(btVector3(0.0f, -9.8f, 0.0f));
   
      /* Add a static plane as the experiment floor on request */
      if(NodeExists(t_tree, "floor")) {
         m_pcGroundCollisionShape = new btStaticPlaneShape(btVector3(0.0f, 1.0f, 0.0f), 0);
         m_pcGroundMotionState = new btDefaultMotionState(btTransform::getIdentity());
         m_pcGroundRigidBody = new btRigidBody(btRigidBody::btRigidBodyConstructionInfo(
            0.0f, m_pcGroundMotionState, m_pcGroundCollisionShape, btVector3(0.0f, 0.0f, 0.0f)));
            
         m_pcWorld->addRigidBody(m_pcGroundRigidBody);
      }
      
      /* Enable ghost objects (used by CDynamics3DEngine::IsLocationOccupied) */
      m_pcGhostPairCallback = new btGhostPairCallback();
      m_pcWorld->getPairCache()->setInternalGhostPairCallback(m_pcGhostPairCallback);
      
      /* Set the random seed from a random number taken from ARGoS RNG */
      //m_pcRNG = CARGoSRandom::CreateRNG("argos");
      
      m_pcBroadphaseInterface->resetPool(m_pcCollisionDispatcher);
		m_pcSolver->setRandSeed(100ul);
   }

   /****************************************/
   /****************************************/

   void CDynamics3DEngine::Reset() {
      
      
      
      /* Remove and reset the physics entities
       * by iterating over the vector, we ensure that the entities are removed in the same order
       * as they were added during initisation
       */
      for(CDynamics3DEntity::TMap::iterator it = m_tPhysicsEntities.begin();
          it != m_tPhysicsEntities.end(); ++it) {
         
         //(*iterVec)->RemoveEntityFromWorld();
         it->second->Reset();
      }
      
      /* clear all forces in the world */
      m_pcWorld->clearForces();
      
      /*
       * Reset the random seed - TODO take from ARGoS RNG
       */     
      m_pcBroadphaseInterface->resetPool(m_pcCollisionDispatcher);
		m_pcSolver->setRandSeed(100ul);

		/* Add elements back to the engine
		 * by iterating over the vector, we ensure that the entities are added in the same order
       * as they were added during initisation, this is important for repeatability between resets
       */
		for(CDynamics3DEntity::TMap::iterator iter = m_tPhysicsEntities.begin();
          iter != m_tPhysicsEntities.end(); ++iter) {
  
         //(*iter)->AddEntityToWorld();
      }
   }
   
   /****************************************/
   /****************************************/

   void CDynamics3DEngine::Destroy() {
      /* empty the physics entity map */
      for(CDynamics3DEntity::TMap::iterator it = m_tPhysicsEntities.begin();
          it != m_tPhysicsEntities.end(); ++it) {
         delete it->second;
      }
      m_tPhysicsEntities.clear();
      
      /* remove the floor if it was added */
      if(m_pcGroundRigidBody != NULL) {
         m_pcWorld->removeRigidBody(m_pcGroundRigidBody);
         delete m_pcGroundRigidBody;
         delete m_pcGroundMotionState;
         delete m_pcGroundCollisionShape;
      }
      
      /* cleanup the dynamics world */
      delete m_pcWorld;
      delete m_pcGhostPairCallback;
      delete m_pcSolver;
      delete m_pcCollisionDispatcher;
      delete m_pcCollisionConfiguration;
      delete m_pcBroadphaseInterface;
   }

   /****************************************/
   /****************************************/

   void CDynamics3DEngine::Update() {
      /* Update the physics state from the entities */
      for(CDynamics3DEntity::TMap::iterator it = m_tPhysicsEntities.begin();
          it != m_tPhysicsEntities.end(); ++it) {
         it->second->UpdateFromEntityStatus();
      }
      
      /* Advance the simulation by m_fSimulationClockTick */
      m_pcWorld->stepSimulation(m_fSimulationClockTick, 0u);

      /* Update the simulated space */
      for(CDynamics3DEntity::TMap::iterator it = m_tPhysicsEntities.begin();
          it != m_tPhysicsEntities.end(); ++it) {
         it->second->UpdateEntityStatus();
      }
   }
   
   /****************************************/
   /****************************************/
   
   bool CDynamics3DEngine::IsRegionOccupied(btTransform& c_transform, btCollisionShape& c_collsion_shape) {      
      bool bRegionOccupied;
      /* Create a ghost object at the specified location with the specified collsion shape */
      btGhostObject cGhostObject;
      cGhostObject.setCollisionShape(&c_collsion_shape);
      cGhostObject.setWorldTransform(c_transform);
      
      m_pcWorld->addCollisionObject(&cGhostObject);
      
      bRegionOccupied = (cGhostObject.getNumOverlappingObjects() != 0);

      m_pcWorld->removeCollisionObject(&cGhostObject);
      return bRegionOccupied;
   }

   /****************************************/
   /****************************************/
   
   UInt32 CDynamics3DEngine::GetNumPhysicsEngineEntities() {
      return m_tPhysicsEntities.size();
   }
     
   /****************************************/
   /****************************************/

   void CDynamics3DEngine::AddEntity(CEntity& c_entity) {
      CallEntityOperation<CDynamics3DOperationAddEntity, CDynamics3DEngine, void>(*this, c_entity);
   }

   /****************************************/
   /****************************************/

   void CDynamics3DEngine::RemoveEntity(CEntity& c_entity) {
      CallEntityOperation<CDynamics3DOperationRemoveEntity, CDynamics3DEngine, void>(*this, c_entity);
   }

   /****************************************/
   /****************************************/

   void CDynamics3DEngine::AddPhysicsEntity(const std::string& str_id,
                                             CDynamics3DEntity& c_entity) {
      m_tPhysicsEntities[str_id] = &c_entity;
      
      for(std::vector<btRigidBody*>::const_iterator itBody = c_entity.GetRigidBodies().begin(); 
          itBody !=  c_entity.GetRigidBodies().end();
          itBody++) {   
         m_pcWorld->addRigidBody(*itBody);
      }
      for(std::vector<btTypedConstraint*>::const_iterator itConstraint = c_entity.GetConstraints().begin(); 
          itConstraint !=  c_entity.GetConstraints().end();
          itConstraint++) {   
         m_pcWorld->addConstraint(*itConstraint, true);
      }
   }

   /****************************************/
   /****************************************/
   void CDynamics3DEngine::RemovePhysicsEntity(const std::string& str_id) {
      CDynamics3DEntity::TMap::iterator it = m_tPhysicsEntities.find(str_id);
      if(it != m_tPhysicsEntities.end()) {
         for(std::vector<btTypedConstraint*>::const_iterator itConstraint = it->second->GetConstraints().begin(); 
             itConstraint !=  it->second->GetConstraints().end();
             itConstraint++) {   
            m_pcWorld->removeConstraint(*itConstraint);
         }
         for(std::vector<btRigidBody*>::const_iterator itBody = it->second->GetRigidBodies().begin(); 
             itBody !=  it->second->GetRigidBodies().end();
             itBody++) {
            m_pcWorld->removeRigidBody(*itBody);
         }
         delete it->second;
         m_tPhysicsEntities.erase(it);
      }
      else {
         THROW_ARGOSEXCEPTION("Dynamics3D entity id \"" << str_id << "\" not found in dynamics 3D engine \"" << GetId() << "\"");
      }
   }

   /****************************************/
   /****************************************/
  
   REGISTER_PHYSICS_ENGINE(CDynamics3DEngine,
                           "dynamics3d",
                           "Michael Allwright [allsey87@gmail.com]",
                           "1.0",
                           "A 3D dynamics physics engine",
                           "Dynamics3D is a plugin based on the bullet physics library",
                           "Under development"
      );
}
