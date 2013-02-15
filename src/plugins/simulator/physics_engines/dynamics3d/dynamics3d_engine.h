/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_engine.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef DYNAMICS3D_ENGINE_H
#define DYNAMICS3D_ENGINE_H

namespace argos {
   class CDynamics3DEngine;
   class CDynamics3DEntity;
}

#include <argos3/core/simulator/entity/controllable_entity.h>
#include <argos3/core/simulator/physics_engine/physics_engine.h>
#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_entity.h>

//#include <argos3/core/utility/math/rng.h>

#include <btBulletDynamicsCommon.h>
#include <BulletCollision/CollisionDispatch/btGhostObject.h>

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
      
      bool IsRegionOccupied(btTransform& c_transform, 
                            btCollisionShape& c_collsion_shape);
      
      void AddPhysicsEntity(const std::string& str_id,
                            CDynamics3DEntity& c_entity);
      void RemovePhysicsEntity(const std::string& str_id);      
   
   private:

      CControllableEntity::TMap m_tControllableEntities;
      CDynamics3DEntity::TMap m_tPhysicsEntities;
      
      /* ARGoS RNG */
      //CARGoSRandom::CRNG* m_pcRNG;
     
      /* Bullet Physics World Data */
      btBroadphaseInterface* m_pcBroadphaseInterface;
      btDefaultCollisionConfiguration* m_pcCollisionConfiguration;
      btCollisionDispatcher* m_pcCollisionDispatcher;
      btSequentialImpulseConstraintSolver* m_pcSolver;
      btDiscreteDynamicsWorld* m_pcWorld;
      btGhostPairCallback* m_pcGhostPairCallback;
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

#define REGISTER_STANDARD_DYNAMICS3D_OPERATION_ADD_ENTITY(SPACE_ENTITY, DYN3D_ENTITY) \
   class CDynamics3DOperationAdd ## SPACE_ENTITY : public CDynamics3DOperationAddEntity { \
   public:                                                              \
   CDynamics3DOperationAdd ## SPACE_ENTITY() {}                         \
   virtual ~CDynamics3DOperationAdd ## SPACE_ENTITY() {}                \
   void ApplyTo(CDynamics3DEngine& c_engine,                            \
                SPACE_ENTITY& c_entity) {                               \
      DYN3D_ENTITY* pcPhysEntity = new DYN3D_ENTITY(c_engine,           \
                                                    c_entity);          \
      c_engine.AddPhysicsEntity(c_entity.GetId(),                       \
                                *pcPhysEntity);                         \
      c_entity.                                                         \
         GetComponent<CEmbodiedEntity>("body").                         \
         AddPhysicsEngine(c_engine);                                    \
      c_entity.                                                         \
         GetComponent<CEmbodiedEntity>("body").                         \
         AddPhysicsEngineEntity(c_engine.GetId(), *pcPhysEntity);       \
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
      c_engine.RemovePhysicsEntity(c_entity.GetId());                   \
      c_entity.                                                         \
         GetComponent<CEmbodiedEntity>("body").                         \
         RemovePhysicsEngine(c_engine);                                 \
      c_entity.                                                         \
         GetComponent<CEmbodiedEntity>("body").                         \
         RemovePhysicsEngineEntity(c_engine.GetId());                   \
   }                                                                    \
   };                                                                   \
   REGISTER_DYNAMICS3D_OPERATION(CDynamics3DOperationRemoveEntity,      \
                                 CDynamics3DOperationRemove ## SPACE_ENTITY, \
                                 SPACE_ENTITY);
   
#define REGISTER_STANDARD_DYNAMICS3D_OPERATIONS_ON_ENTITY(SPACE_ENTITY, DYN3D_ENTITY) \
   REGISTER_STANDARD_DYNAMICS3D_OPERATION_ADD_ENTITY(SPACE_ENTITY, DYN3D_ENTITY) \
   REGISTER_STANDARD_DYNAMICS3D_OPERATION_REMOVE_ENTITY(SPACE_ENTITY)

   /****************************************/
   /****************************************/

}

#endif
