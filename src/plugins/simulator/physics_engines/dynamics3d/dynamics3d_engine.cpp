/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_engine.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "dynamics3d_engine.h"

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/entity/embodied_entity.h>

#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_model.h>
#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_body.h>

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
      m_pcGroundRigidBody(NULL),
      m_unIterations(10) {}

   /****************************************/
   /****************************************/

   void CDynamics3DEngine::Init(TConfigurationNode& t_tree) {
      /* Init parent */
      CPhysicsEngine::Init(t_tree);
      
      /* Parse the XML */
      GetNodeAttributeOrDefault(t_tree, "iterations", m_unIterations, m_unIterations);
      m_fDeltaT = m_fSimulationClockTick / (Real)m_unIterations;

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
      for(std::vector<std::pair<std::string, CDynamics3DModel*> >::iterator itModel = m_vecPhysicsModels.begin();
          itModel != m_vecPhysicsModels.end(); ++itModel) {
         
         RemoveConstraintsFromModel(*itModel->second);
         RemoveBodiesFromModel(*itModel->second);
         itModel->second->Reset();
      }
      
      //@todo create a method/struct that contrains/creates and resets the floor
      m_pcGroundRigidBody->clearForces();
      
      /* clear all forces in the world */
      m_pcWorld->clearForces();
      
      /* Reset the random seed */
      //@todo take the seed from ARGoS RNG */     
      m_pcBroadphaseInterface->resetPool(m_pcCollisionDispatcher);
      m_pcSolver->setRandSeed(100ul);

      /* Add elements back to the engine
       * by iterating over the vector, we ensure that the entities are added in the same order
       * as they were added during initisation, this is important for repeatability between resets
       */
      for(std::vector<std::pair<std::string, CDynamics3DModel*> >::iterator itModel = m_vecPhysicsModels.begin();
          itModel != m_vecPhysicsModels.end(); ++itModel) {
  
         AddBodiesFromModel(*itModel->second);
         AddConstraintsFromModel(*itModel->second);
      }
   }
   
   /****************************************/
   /****************************************/

   void CDynamics3DEngine::Destroy() {
      /* empty the physics entity map */
      for(std::vector<std::pair<std::string, CDynamics3DModel*> >::iterator it = m_vecPhysicsModels.begin();
          it != m_vecPhysicsModels.end(); ++it) {
         delete it->second;
      }
      m_vecPhysicsModels.clear();
      
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
      for(std::vector<std::pair<std::string, CDynamics3DModel*> >::iterator it = m_vecPhysicsModels.begin();
          it != m_vecPhysicsModels.end(); ++it) {
         it->second->UpdateFromEntityStatus();
      }

      /* Step the simuation forwards */
      m_pcWorld->stepSimulation(m_fSimulationClockTick, m_unIterations, m_fDeltaT);

      /* Update the simulated space */
      for(std::vector<std::pair<std::string, CDynamics3DModel*> >::iterator it = m_vecPhysicsModels.begin();
          it != m_vecPhysicsModels.end(); ++it) {
         it->second->CalculateBoundingBox();
         it->second->UpdateEntityStatus();
         
      }
   }

   bool CDynamics3DEngine::IsModelCollidingWithSomething(const CDynamics3DModel& c_model) {
      /* this doesn't step the simulation, but rather reruns the collision detection */
      m_pcWorld->performDiscreteCollisionDetection();

      /* get the vector of bodies associated with the given model */
      const CDynamics3DBody::TVector& vecModelBodies = c_model.GetBodies();

      // an iterator over the model
      CDynamics3DBody::TVector::const_iterator itBodyConfiguration;
      
      for(UInt32 i = 0; i < UInt32(m_pcCollisionDispatcher->getNumManifolds()); i++) {
         
         btPersistentManifold* pcContactManifold = m_pcCollisionDispatcher->getManifoldByIndexInternal(i);
         const btCollisionObject* pcBodyA = pcContactManifold->getBody0();
         const btCollisionObject* pcBodyB = pcContactManifold->getBody1();
         
         bool bBelongsToModelBodyA = false;
         bool bBelongsToModelBodyB = false;

         // ignore collisions with the ground
         if(m_pcGroundRigidBody == pcBodyA || m_pcGroundRigidBody == pcBodyB) {
            continue;
         }
        
         // Check if either body in the contact manifold belongs to the model
         for(itBodyConfiguration = vecModelBodies.begin();
             itBodyConfiguration != vecModelBodies.end();
             ++itBodyConfiguration) {
            
            if(itBodyConfiguration->m_pcRigidBody == pcBodyA) {
               bBelongsToModelBodyA = true;
            }
            if(itBodyConfiguration->m_pcRigidBody == pcBodyB) {
               bBelongsToModelBodyB = true;
            }
            //@todo optimisation: once both are true we can exit this loop
         }

         // if the collision pair exists within the same model, ignore it!
         if(bBelongsToModelBodyA == true && bBelongsToModelBodyB == true) {
            continue;
         }
         
         // if niether body in the collision pair belongs to this model, ignore it!
         if(bBelongsToModelBodyA == false && bBelongsToModelBodyB == false) {
            continue;
         }

         /* At this point we know that one of the two bodies involved in the contact manifold
            belong to this model, we now check for contact points with negative distance to 
            indicate a collision */
         for(UInt32 j = 0; j < UInt32(pcContactManifold->getNumContacts()); j++) {
            
            btManifoldPoint& cManifoldPoint = pcContactManifold->getContactPoint(j);
            if (cManifoldPoint.getDistance() < 0.0f) {
               // This manifold tells us that the model is coliding with something
               // We can return true
               return true;
            }
         }
      }
      return false;
   }
   
   /****************************************/
   /****************************************/
   
   UInt32 CDynamics3DEngine::GetNumPhysicsEngineEntities() {
      return m_vecPhysicsModels.size();
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

   void CDynamics3DEngine::AddPhysicsModel(const std::string& str_id,
                                           CDynamics3DModel& c_model) {
      //@todo check for duplicates?
      m_vecPhysicsModels.push_back(std::pair<std::string, CDynamics3DModel*>(str_id, &c_model));
      AddBodiesFromModel(c_model);      
      AddConstraintsFromModel(c_model);
      
   }

   /****************************************/
   /****************************************/

   void CDynamics3DEngine::RemovePhysicsModel(const std::string& str_id) {
      std::vector<std::pair<std::string, CDynamics3DModel*> >::iterator itModel = m_vecPhysicsModels.Find(str_id);
      if(itModel != m_vecPhysicsModels.end()) {
         RemoveConstraintsFromModel(*itModel->second);
         RemoveBodiesFromModel(*itModel->second);
         delete itModel->second;
         m_vecPhysicsModels.erase(itModel);
      }
      else {
         THROW_ARGOSEXCEPTION("Dynamics3D model id \"" << str_id << "\" not found in dynamics 3D engine \"" << GetId() << "\"");
      }
   }

   /****************************************/
   /****************************************/


   void CDynamics3DEngine::AddBodiesFromModel(const CDynamics3DModel& c_model) {
      const CDynamics3DBody::TVector& vecModelBodies = c_model.GetBodies();

      for(CDynamics3DBody::TVector::const_iterator itBodyConfiguration = vecModelBodies.begin(); 
          itBodyConfiguration !=  vecModelBodies.end();
          itBodyConfiguration++) {   
         m_pcWorld->addRigidBody(itBodyConfiguration->m_pcRigidBody);
      }
   }

   /****************************************/
   /****************************************/

   void CDynamics3DEngine::AddConstraintsFromModel(const CDynamics3DModel& c_model) {
      const std::vector<CDynamics3DModel::SConstraint>& vecModelConstraints = c_model.GetConstraints();

      for(std::vector<CDynamics3DModel::SConstraint>::const_iterator itConstraint = vecModelConstraints.begin(); 
          itConstraint != vecModelConstraints.end(); 
          itConstraint++) {
         m_pcWorld->addConstraint(itConstraint->m_pcConstraint, itConstraint->m_bDisableCollisions);
      }
   }

   /****************************************/
   /****************************************/

   void CDynamics3DEngine::RemoveConstraintsFromModel(const CDynamics3DModel& c_model) {
      for(std::vector<CDynamics3DModel::SConstraint>::const_iterator itConstraint = c_model.GetConstraints().begin(); 
          itConstraint !=  c_model.GetConstraints().end();
          itConstraint++) {
         m_pcWorld->removeConstraint(itConstraint->m_pcConstraint);
      }
   }

   /****************************************/
   /****************************************/
   
   void CDynamics3DEngine::RemoveBodiesFromModel(const CDynamics3DModel& c_model) {
      for(CDynamics3DBody::TVector::const_iterator itBody = c_model.GetBodies().begin(); 
          itBody !=  c_model.GetBodies().end();
          itBody++) {
         m_pcWorld->removeRigidBody(itBody->m_pcRigidBody);
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
                           "Under development");
}
