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
#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_plugin.h>

#include <algorithm>

namespace argos {

   /****************************************/
   /****************************************/   

   btStaticPlaneShape CDynamics3DEngine::m_cGroundCollisionShape(btVector3(0.0f, 1.0f, 0.0f), 0);

   /****************************************/
   /****************************************/

   CDynamics3DEngine::CDynamics3DEngine() :
      m_pcRNG(NULL),
      m_cRandomSeedRange(0,1000),
      m_pcBroadphaseInterface(NULL),
      m_pcCollisionConfiguration(NULL),
      m_pcCollisionDispatcher(NULL),
      m_pcSolver(NULL),
      m_pcWorld(NULL),
      m_pcGround(NULL),
      m_unIterations(10),
      m_bEntityTransferActive(false) {}

   /****************************************/
   /****************************************/

   void CDynamics3DEngine::Init(TConfigurationNode& t_tree) {
      /* Init parent */
      CPhysicsEngine::Init(t_tree);
      /* create the random number generator */
      m_pcRNG = CRandom::CreateRNG("argos");      
      /* Parse the XML */
      GetNodeAttributeOrDefault(t_tree, "iterations", m_unIterations, m_unIterations);
      m_fDeltaT = GetPhysicsClockTick() / (Real)m_unIterations;
      /* Select the default broadphase, collision configuration, dispatcher and solver */
      m_pcBroadphaseInterface = new btDbvtBroadphase;
      m_pcCollisionConfiguration = new btDefaultCollisionConfiguration;
      m_pcCollisionDispatcher = new btCollisionDispatcher(m_pcCollisionConfiguration);
      m_pcSolver = new btSequentialImpulseConstraintSolver;
      /* Create the physics world */
      m_pcWorld = new btDiscreteDynamicsWorld(m_pcCollisionDispatcher,
                                              m_pcBroadphaseInterface,
                                              m_pcSolver,
                                              m_pcCollisionConfiguration);
      /* Set the gravity in the world */
      m_pcWorld->setGravity(btVector3(0.0f, -9.8f, 0.0f));
      /* clear the forces in the world (shouldn't  be required as there are no bodies in the world) */
      m_pcWorld->clearForces();
      /* reset the solvers and dispatchers */
      m_pcBroadphaseInterface->resetPool(m_pcCollisionDispatcher);
      m_pcSolver->setRandSeed(m_pcRNG->Uniform(m_cRandomSeedRange));
      /* Add a static plane as the experiment floor on request */
      if(NodeExists(t_tree, "floor")) {
         m_pcGround = new CDynamics3DBody(NULL, "floor", &m_cGroundCollisionShape);
         m_pcGround->AddBodyToWorld(m_pcWorld);
      }
      /* load the plugins */
      if(NodeExists(t_tree, "plugins")) {
         TConfigurationNodeIterator itPlugin;
         for(itPlugin = itPlugin.begin(&GetNode(t_tree, "plugins"));
             itPlugin != itPlugin.end();
             ++itPlugin) {
            CDynamics3DPlugin* pcPlugin = CFactory<CDynamics3DPlugin>::New(itPlugin->Value());
            pcPlugin->SetEngine(*this);
            pcPlugin->Init(*itPlugin);
            AddPhysicsPlugin(*pcPlugin);
         }
      }
      /* Parse the boundaries of this physics engine if they exist */
      if(NodeExists(t_tree, "boundaries")) {
         /* Parse the boundary definition */
         TConfigurationNode& tBoundaries = GetNode(t_tree, "boundaries");
         SBoundarySegment sBoundSegment;
         CVector2 cLastPoint, cCurPoint;
         std::string strConnectWith;
         TConfigurationNodeIterator tVertexIt("vertex");
         /* Get the first vertex */
         tVertexIt = tVertexIt.begin(&tBoundaries);
         if(tVertexIt == tVertexIt.end()) {
            THROW_ARGOSEXCEPTION("Physics engine of type \"dynamics3d\", id \"" << GetId() << "\": you didn't specify any <vertex>!");
         }
         GetNodeAttribute(*tVertexIt, "point", cLastPoint);
         m_vecVertices.push_back(cLastPoint);
         /* Go through the other vertices */
         ++tVertexIt;
         while(tVertexIt != tVertexIt.end()) {
            /* Read vertex data and fill in segment struct */
            GetNodeAttribute(*tVertexIt, "point", cCurPoint);
            m_vecVertices.push_back(cCurPoint);
            sBoundSegment.Segment.SetStart(cLastPoint);
            sBoundSegment.Segment.SetEnd(cCurPoint);
            GetNodeAttribute(*tVertexIt, "connect_with", strConnectWith);
            if(strConnectWith == "gate") {
               /* Connect to previous vertex with a gate */
               sBoundSegment.Type = SBoundarySegment::SEGMENT_TYPE_GATE;
               GetNodeAttribute(*tVertexIt, "to_engine", sBoundSegment.EngineId);
            }
            else if(strConnectWith == "wall") {
               /* Connect to previous vertex with a wall */
               sBoundSegment.Type = SBoundarySegment::SEGMENT_TYPE_WALL;
               sBoundSegment.EngineId = "";
            }
            else {
               /* Parse error */
               THROW_ARGOSEXCEPTION("Physics engine of type \"dynamics3d\", id \"" << GetId() << "\": unknown vertex connection method \"" << strConnectWith << "\". Allowed methods are \"wall\" and \"gate\".");
            }
            m_vecSegments.push_back(sBoundSegment);
            /* Next vertex */
            cLastPoint = cCurPoint;
            ++tVertexIt;
         }
         /* Check that the boundary is a closed path */
         if(m_vecVertices.front() != m_vecVertices.back()) {
            THROW_ARGOSEXCEPTION("Physics engine of type \"dynamics3d\", id \"" << GetId() << "\": the specified path is not closed. The first and last points of the boundaries MUST be the same.");
         }
      }
   }

   /****************************************/
   /****************************************/

   void CDynamics3DEngine::Reset() {
      /* Remove and reset the physics entities
       * by iterating over the vector, we ensure that the entities are removed in the same order
       * as they were added during initisation
       */
      for(CDynamics3DModel::TVector::iterator itModel = m_vecPhysicsModels.begin();
          itModel != m_vecPhysicsModels.end();
          ++itModel) {         
         RemoveJointsFromModel(**itModel);
         RemoveBodiesFromModel(**itModel);
         (*itModel)->Reset();
      }
      if(m_pcGround != NULL) {
         m_pcGround->RemoveBodyFromWorld(m_pcWorld);
         m_pcGround->Reset();
      }
      /* clear the forces in the world (shouldn't  be required as there are no bodies in the world) */
      m_pcWorld->clearForces();
      /* reset the solvers and dispatchers */
      m_pcBroadphaseInterface->resetPool(m_pcCollisionDispatcher);
      m_pcSolver->setRandSeed(m_pcRNG->Uniform(m_cRandomSeedRange));
      /* Add elements back to the engine
       * by iterating over the vector, we ensure that the entities are added in the same order
       * as they were added during initisation, this is important for repeatability between resets
       */
      if(m_pcGround != NULL) {
         m_pcGround->AddBodyToWorld(m_pcWorld);
      }
      for(CDynamics3DModel::TVector::iterator itModel = m_vecPhysicsModels.begin();
          itModel != m_vecPhysicsModels.end();
          ++itModel) {
         AddBodiesFromModel(**itModel);
         AddJointsFromModel(**itModel);
      }
   }
   
   /****************************************/
   /****************************************/

   void CDynamics3DEngine::Destroy() {
      /* empty the physics entity map */
      for(CDynamics3DModel::TVector::iterator itModel = m_vecPhysicsModels.begin();
          itModel != m_vecPhysicsModels.end();
          ++itModel) {
         delete *itModel;
      }
      m_vecPhysicsModels.clear();      
      /* remove the floor if it was added */
      if(m_pcGround != NULL) {
         m_pcGround->RemoveBodyFromWorld(m_pcWorld);
         delete m_pcGround;
      }      
      /* delete the dynamics world */
      delete m_pcWorld;
      delete m_pcSolver;
      delete m_pcCollisionDispatcher;
      delete m_pcCollisionConfiguration;
      delete m_pcBroadphaseInterface;
   }

   /****************************************/
   /****************************************/

   void CDynamics3DEngine::Update() {      
      /* Update the physics state from the entities */
      for(CDynamics3DModel::TVector::iterator itModel = m_vecPhysicsModels.begin();
          itModel != m_vecPhysicsModels.end();
          ++itModel) {
         (*itModel)->UpdateFromEntityStatus();
      }
      /* Execute the plugins update methods */
      for(CDynamics3DPlugin::TVector::iterator itPlugin = m_vecPhysicsPlugins.begin();
          itPlugin != m_vecPhysicsPlugins.end();
          ++itPlugin) {
         (*itPlugin)->Update();
      }
      /* Step the simuation forwards */
      m_pcWorld->stepSimulation(GetPhysicsClockTick(), m_unIterations, m_fDeltaT);
      /////
      //fprintf(stderr, "m_fSimulationClockTick = %.8f, m_unIterations = %d, m_fDeltaT = %.8f\n", m_fSimulationClockTick, m_unIterations, m_fDeltaT);
      /*
      	btDefaultSerializer*	serializer = new btDefaultSerializer();
	m_pcWorld->serialize(serializer);
 
	FILE* file = fopen("argos3dworld.bullet","wb");
	fwrite(serializer->getBufferPointer(),serializer->getCurrentBufferSize(),1, file);
	fclose(file);
      */
        ///
      /* Update the simulated space */
      for(CDynamics3DModel::TVector::iterator itModel = m_vecPhysicsModels.begin();
          itModel != m_vecPhysicsModels.end();
          ++itModel) {
         (*itModel)->CalculateBoundingBox();
         (*itModel)->UpdateEntityStatus();
      }
   }
   
   /****************************************/
   /****************************************/
   
   CEmbodiedEntity* CDynamics3DEngine::CheckIntersectionWithRay(Real& f_t_on_ray,
                                                                const CRay3& c_ray) const {
      btVector3 cRayStart(ARGoSToBullet(c_ray.GetStart()));
      btVector3 cRayEnd(ARGoSToBullet(c_ray.GetEnd()));

      btCollisionWorld::ClosestRayResultCallback cResult(cRayStart, cRayEnd);
   
      m_pcWorld->rayTest(cRayStart, cRayEnd, cResult);

      if (cResult.hasHit()) {
         f_t_on_ray = (cResult.m_hitPointWorld - cRayStart).length() / c_ray.GetLength();
         CDynamics3DBody* pcBody = static_cast<CDynamics3DBody*>(cResult.m_collisionObject->getUserPointer());
         return &(pcBody->GetParentModel().GetEmbodiedEntity());
      }

      f_t_on_ray = 0.0f;
      return NULL;
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

   bool CDynamics3DEngine::IsModelCollidingWithSomething(const CDynamics3DModel& c_model) {
      /* this doesn't step the simulation, but rather reruns the collision detection */
      /* @todo: this method is very slow and memory consuming, find an alternative */
      m_pcWorld->performDiscreteCollisionDetection();
      /* get the vector of bodies associated with the given model */
      const CDynamics3DBody::TVector& vecModelBodies = c_model.GetBodies();
      // an iterator over the model
      CDynamics3DBody::TVector::const_iterator itBody;
      for(UInt32 i = 0; i < UInt32(m_pcCollisionDispatcher->getNumManifolds()); i++) {
         btPersistentManifold* pcContactManifold = m_pcCollisionDispatcher->getManifoldByIndexInternal(i);
         const btCollisionObject* pcBodyA = pcContactManifold->getBody0();
         const btCollisionObject* pcBodyB = pcContactManifold->getBody1();
         bool bBelongsToModelBodyA = false;
         bool bBelongsToModelBodyB = false;
         // ignore collisions with the ground
         if(*m_pcGround == pcBodyA || *m_pcGround == pcBodyB) {
            continue;
         }        
         // Check if either body in the contact manifold belongs to the model
         for(itBody = vecModelBodies.begin();
             itBody != vecModelBodies.end();
             ++itBody) {  
            if(**itBody == pcBodyA) {
               bBelongsToModelBodyA = true;
            }
            if(**itBody == pcBodyB) {
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
               // We can now return true
               return true;
            }
         }
      }
      return false;
   }
   
   /****************************************/
   /****************************************/

   void CDynamics3DEngine::AddPhysicsPlugin(CDynamics3DPlugin& c_plugin) {
      m_vecPhysicsPlugins.push_back(&c_plugin);
   }

   /****************************************/
   /****************************************/

   void CDynamics3DEngine::RemovePhysicsPlugin(const std::string& str_id) {

      CDynamics3DPlugin::TVector::iterator itPlugin = std::find(m_vecPhysicsPlugins.begin(),
                                                                m_vecPhysicsPlugins.end(),
                                                                str_id);
      if(itPlugin != m_vecPhysicsPlugins.end()) {
         delete *itPlugin;
         m_vecPhysicsPlugins.erase(itPlugin);
      }
      else {
         THROW_ARGOSEXCEPTION("Dynamics3D plugin id \"" << str_id <<
                              "\" not found in dynamics 3D engine \"" << GetId() << "\"");
      }
   }

   /****************************************/
   /****************************************/

   void CDynamics3DEngine::AddPhysicsModel(CDynamics3DModel& c_model) {
      //@todo check for duplicates?
      m_vecPhysicsModels.push_back(&c_model);
      AddBodiesFromModel(c_model);      
      AddJointsFromModel(c_model);
      // Notify the plugins of the added model
      for(CDynamics3DPlugin::TVector::iterator itPlugin = m_vecPhysicsPlugins.begin();
          itPlugin != m_vecPhysicsPlugins.end();
          ++itPlugin) {
         (*itPlugin)->RegisterModel(c_model);
      }
   }

   /****************************************/
   /****************************************/

   void CDynamics3DEngine::RemovePhysicsModel(const std::string& str_id) {

      CDynamics3DModel::TVector::iterator itModel = std::find(m_vecPhysicsModels.begin(),
                                                              m_vecPhysicsModels.end(),
                                                              str_id);
      if(itModel != m_vecPhysicsModels.end()) {
         // Notify the plugins of model removal
         for(CDynamics3DPlugin::TVector::iterator itPlugin = m_vecPhysicsPlugins.begin();
             itPlugin != m_vecPhysicsPlugins.end();
             ++itPlugin) {
            (*itPlugin)->UnregisterModel(**itModel);
         }
         RemoveJointsFromModel(**itModel);
         RemoveBodiesFromModel(**itModel);
         delete *itModel;
         m_vecPhysicsModels.erase(itModel);
      }
      else {
         THROW_ARGOSEXCEPTION("Dynamics3D model id \"" << str_id <<
                              "\" not found in dynamics 3D engine \"" << GetId() << "\"");
      }
   }

   /****************************************/
   /****************************************/

   bool CDynamics3DEngine::IsPointContained(const CVector3& c_point) {
      if(!IsEntityTransferActive()) {
         /* The engine has no boundaries on XY, so the wanted point is in for sure */
         return true;
      }
      else {
         /* Check the boundaries */
         for(size_t i = 0; i < m_vecSegments.size(); ++i) {
            const CVector2& cP0 = m_vecSegments[i].Segment.GetStart();
            const CVector2& cP1 = m_vecSegments[i].Segment.GetEnd();
            Real fCriterion =
               (c_point.GetY() - cP0.GetY()) * (cP1.GetX() - cP0.GetX()) -
               (c_point.GetX() - cP0.GetX()) * (cP1.GetY() - cP0.GetY());
            if(fCriterion > 0.0f) {
               return false;
            }
         }
         return true;
      }
   }

   /****************************************/
   /****************************************/

   void CDynamics3DEngine::TransferEntities() {
      for(size_t i = 0; i < m_vecTransferData.size(); ++i) {
         CPhysicsEngine& cToEngine = CSimulator::GetInstance().GetPhysicsEngine(m_vecTransferData[i].EngineId);
         /* @todo: we need to extend the add entity method to recieve information about 
          * velocity of individual bodies
          */
         cToEngine.AddEntity(*m_vecTransferData[i].Entity);
         RemoveEntity(*m_vecTransferData[i].Entity);
      }
      m_vecTransferData.clear();
   }

   /****************************************/
   /****************************************/

   bool CDynamics3DEngine::CalculateTransfer(Real f_x,
                                             Real f_y,
                                             std::string& str_engine_id) {
      /*
       * @todo: this method makes the assumption that only one gate is trespassed at any time.
       * This assumption may be false in some ill-shaped polygons or when the gate isn't just a
       * segment, but is a sequence of segments.
       */
      for(size_t i = 0; i < m_vecSegments.size(); ++i) {
         if(m_vecSegments[i].Type == SBoundarySegment::SEGMENT_TYPE_GATE) {
            const CVector2& cP0 = m_vecSegments[i].Segment.GetStart();
            const CVector2& cP1 = m_vecSegments[i].Segment.GetEnd();
            Real fCriterion =
               (f_y - cP0.GetY()) * (cP1.GetX() - cP0.GetX()) -
               (f_x - cP0.GetX()) * (cP1.GetY() - cP0.GetY());
            if(fCriterion < 0.0f) {
               str_engine_id = m_vecSegments[i].EngineId;
               return true;
            }
         }
      }
      return false;
   }

   /****************************************/
   /****************************************/

   void CDynamics3DEngine::ScheduleEntityForTransfer(CEntity& c_entity,
                                                     const std::string& str_engine_id) {
      m_vecTransferData.push_back(SEntityTransferData());
      m_vecTransferData.back().EngineId = str_engine_id;
      m_vecTransferData.back().Entity = &c_entity;
   }

   /****************************************/
   /****************************************/

   void CDynamics3DEngine::AddBodiesFromModel(CDynamics3DModel& c_model) {
      for(CDynamics3DBody::TVector::iterator itBody = c_model.GetBodies().begin(); 
          itBody != c_model.GetBodies().end();
          ++itBody) {   
         (*itBody)->AddBodyToWorld(m_pcWorld);
      }
   }

   /****************************************/
   /****************************************/

   void CDynamics3DEngine::AddJointsFromModel(CDynamics3DModel& c_model) {
      for(CDynamics3DJoint::TVector::iterator itJoint = c_model.GetJoints().begin(); 
          itJoint != c_model.GetJoints().end();
          itJoint++) {
         (*itJoint)->AddJointToWorld(m_pcWorld);
      }
   }

   /****************************************/
   /****************************************/

   void CDynamics3DEngine::RemoveJointsFromModel(CDynamics3DModel& c_model) {
      for(CDynamics3DJoint::TVector::iterator itJoint = c_model.GetJoints().begin();
          itJoint != c_model.GetJoints().end();
          itJoint++) {
         (*itJoint)->RemoveJointFromWorld(m_pcWorld);
      }
   }

   /****************************************/
   /****************************************/
   
   void CDynamics3DEngine::RemoveBodiesFromModel(CDynamics3DModel& c_model) {
      for(CDynamics3DBody::TVector::iterator itBody = c_model.GetBodies().begin(); 
          itBody !=  c_model.GetBodies().end();
          itBody++) {
         (*itBody)->RemoveBodyFromWorld(m_pcWorld);
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
