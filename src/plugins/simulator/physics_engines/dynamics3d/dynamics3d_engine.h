/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_engine.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef DYNAMICS3D_ENGINE_H
#define DYNAMICS3D_ENGINE_H

namespace argos {
   class CDynamics3DEngine;
   class CDynamics3DModel;
}

#include <argos3/core/simulator/entity/controllable_entity.h>
#include <argos3/core/simulator/physics_engine/physics_engine.h>

//#include <argos3/core/utility/math/rng.h>

#include <argos3/plugins/simulator/physics_engines/dynamics3d/bullet/btBulletDynamicsCommon.h>
#include <argos3/plugins/simulator/physics_engines/dynamics3d/bullet/BulletCollision/CollisionDispatch/btGhostObject.h>

namespace argos {

   /****************************************/
   /****************************************/

   class CDynamics3DEngine : public CPhysicsEngine {
   
   public:
      
      CDynamics3DEngine();
      virtual ~CDynamics3DEngine() {}

      virtual void Init(TConfigurationNode& t_tree);
      virtual void Reset();
      virtual void Destroy();

      virtual void Update();

      virtual UInt32 GetNumPhysicsEngineEntities();
      virtual void AddEntity(CEntity& c_entity);
      virtual void RemoveEntity(CEntity& c_entity);
      
      bool IsModelCollidingWithSomething(const CDynamics3DModel& c_model);
      
      void AddPhysicsModel(const std::string& str_id,
                           CDynamics3DModel& c_model);
      
      void RemovePhysicsModel(const std::string& str_id);

      virtual bool IsPointContained(const CVector3& c_point) {
         /** @todo Implement physics boundaries */
         return true;
      }

      virtual bool IsEntityTransferNeeded() const {
         /** @todo Implement physics boundaries */
         return false;
      }

      virtual void TransferEntities() {
         /** @todo Implement physics boundaries */
      }
   
   private:

      void AddBodiesFromModel(CDynamics3DModel& c_model);
      void AddConstraintsFromModel(CDynamics3DModel& c_model);

      void RemoveConstraintsFromModel(CDynamics3DModel& c_model);
      void RemoveBodiesFromModel(CDynamics3DModel& c_model);

   private:

      //@question Is there any reason not to put the actual models into the vector?
      class : public std::vector<std::pair<std::string, CDynamics3DModel*> > {
      public:
         std::vector<std::pair<std::string, CDynamics3DModel*> >::iterator Find(const std::string& str_id) {
            std::vector<std::pair<std::string, CDynamics3DModel*> >::iterator it;
            for(it = this->begin(); it != this->end(); ++it) {
               if(it->first == str_id) break;
            }
            return it;
         }
      } m_vecPhysicsModels;

      /* ARGoS RNG */
      //CARGoSRandom::CRNG* m_pcRNG;
     
      /* Bullet Physics World Data */
      btBroadphaseInterface*                 m_pcBroadphaseInterface;
      btDefaultCollisionConfiguration*       m_pcCollisionConfiguration;
      btCollisionDispatcher*                 m_pcCollisionDispatcher;
      btSequentialImpulseConstraintSolver*   m_pcSolver;
      btDiscreteDynamicsWorld*               m_pcWorld;
      btGhostPairCallback*                   m_pcGhostPairCallback;
      
      /* Dynamics World Floor Data */
      btStaticPlaneShape*                    m_pcGroundCollisionShape;
      btDefaultMotionState*                  m_pcGroundMotionState;
      btRigidBody*                           m_pcGroundRigidBody; 

      size_t m_unIterations;
      Real m_fDeltaT;
      
   };
 
   /****************************************/
   /****************************************/

   template <typename ACTION>
   class CDynamics3DOperation : public CEntityOperation<ACTION, CDynamics3DEngine, void> {
   public:
      virtual ~CDynamics3DOperation() {}
   };

   class CDynamics3DOperationAddEntity : public CDynamics3DOperation<CDynamics3DOperationAddEntity> {
   public:
      virtual ~CDynamics3DOperationAddEntity() {}
   };

   class CDynamics3DOperationRemoveEntity : public CDynamics3DOperation<CDynamics3DOperationRemoveEntity> {
   public:
      virtual ~CDynamics3DOperationRemoveEntity() {}
   };

#define REGISTER_DYNAMICS3D_OPERATION(ACTION, OPERATION, ENTITY)        \
   REGISTER_ENTITY_OPERATION(ACTION, CDynamics3DEngine, OPERATION, void, ENTITY);

#define REGISTER_STANDARD_DYNAMICS3D_OPERATION_ADD_ENTITY(SPACE_ENTITY, DYN3D_MODEL) \
   class CDynamics3DOperationAdd ## SPACE_ENTITY : public CDynamics3DOperationAddEntity { \
   public:                                                              \
   CDynamics3DOperationAdd ## SPACE_ENTITY() {}                         \
   virtual ~CDynamics3DOperationAdd ## SPACE_ENTITY() {}                \
   void ApplyTo(CDynamics3DEngine& c_engine,                            \
                SPACE_ENTITY& c_entity) {                               \
      DYN3D_MODEL* pcPhysModel = new DYN3D_MODEL(c_engine,              \
                                                  c_entity);            \
      c_engine.AddPhysicsModel(c_entity.GetId(),                        \
                                *pcPhysModel);                          \
      c_entity.                                                         \
         GetComponent<CEmbodiedEntity>("body").                         \
         AddPhysicsModel(c_engine.GetId(), *pcPhysModel);               \
   }                                                                    \
   };                                                                   \
   REGISTER_DYNAMICS3D_OPERATION(CDynamics3DOperationAddEntity,         \
                                 CDynamics3DOperationAdd ## SPACE_ENTITY, \
                                 SPACE_ENTITY);
   
#define REGISTER_STANDARD_DYNAMICS3D_OPERATION_REMOVE_ENTITY(SPACE_ENTITY) \
   class CDynamics3DOperationRemove ## SPACE_ENTITY : public CDynamics3DOperationRemoveEntity { \
   public:                                                              \
   CDynamics3DOperationRemove ## SPACE_ENTITY() {}                      \
   virtual ~CDynamics3DOperationRemove ## SPACE_ENTITY() {}             \
   void ApplyTo(CDynamics3DEngine& c_engine,                            \
                SPACE_ENTITY& c_entity) {                               \
                                                                        \
      c_engine.RemovePhysicsModel(c_entity.GetId());                    \
      c_entity.                                                         \
         GetComponent<CEmbodiedEntity>("body").                         \
         RemovePhysicsModel(c_engine.GetId());                          \
   }                                                                    \
   };                                                                   \
   REGISTER_DYNAMICS3D_OPERATION(CDynamics3DOperationRemoveEntity,      \
                                 CDynamics3DOperationRemove ## SPACE_ENTITY, \
                                 SPACE_ENTITY);
   
#define REGISTER_STANDARD_DYNAMICS3D_OPERATIONS_ON_ENTITY(SPACE_ENTITY, DYN3D_MODEL) \
   REGISTER_STANDARD_DYNAMICS3D_OPERATION_ADD_ENTITY(SPACE_ENTITY, DYN3D_MODEL) \
   REGISTER_STANDARD_DYNAMICS3D_OPERATION_REMOVE_ENTITY(SPACE_ENTITY)

}

#endif
