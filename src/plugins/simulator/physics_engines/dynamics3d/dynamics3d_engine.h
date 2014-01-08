/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_engine.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef DYNAMICS3D_ENGINE_H
#define DYNAMICS3D_ENGINE_H

namespace argos {
   class CDynamics3DEngine;
   class CDynamics3DBody;
   class CDynamics3DModel;
   class CDynamics3DPlugin;
}

#include <argos3/core/simulator/entity/controllable_entity.h>
#include <argos3/core/simulator/physics_engine/physics_engine.h>

#include <argos3/core/utility/math/ray2.h>
#include <argos3/core/utility/math/rng.h>

#include <argos3/plugins/simulator/physics_engines/dynamics3d/bullet/btBulletDynamicsCommon.h>
#include <argos3/plugins/simulator/physics_engines/dynamics3d/bullet/BulletCollision/CollisionDispatch/btGhostObject.h>

namespace argos {

   /****************************************/
   /****************************************/

   class CDynamics3DEngine : public CPhysicsEngine {

   public:

      struct SBoundarySegment {
         //@todo segment type will become a face in a true 3D implementation
         CRay2 Segment;
         enum {
            SEGMENT_TYPE_WALL,
            SEGMENT_TYPE_GATE
         } Type;
         std::string EngineId;

         SBoundarySegment() : 
            Type(SEGMENT_TYPE_WALL) {}
      };

      struct SEntityTransferData {
         std::string EngineId;
         CEntity* Entity;
         //@todo add information about multiple bodies, velocities etc

         SEntityTransferData() :
            Entity(NULL) {}
      };

   public:
      
      CDynamics3DEngine();
      virtual ~CDynamics3DEngine() {}

      virtual void Init(TConfigurationNode& t_tree);
      virtual void Reset();
      virtual void Destroy();
      virtual void Update();
      virtual CEmbodiedEntity* CheckIntersectionWithRay(Real& f_t_on_ray,
                                                        const CRay3& c_ray) const;
      virtual UInt32 GetNumPhysicsEngineEntities();

      virtual void AddEntity(CEntity& c_entity);
      virtual void RemoveEntity(CEntity& c_entity);

      bool IsModelCollidingWithSomething(const CDynamics3DModel& c_model);
      
      void AddPhysicsModel(CDynamics3DModel& c_model);      
      void RemovePhysicsModel(const std::string& str_id);

      void AddPhysicsPlugin(CDynamics3DPlugin& c_plugin);
      void RemovePhysicsPlugin(const std::string& str_id);

      std::vector<CDynamics3DModel*>& GetModels() {
         return m_vecPhysicsModels;
      }
      

      virtual bool IsPointContained(const CVector3& c_point);
      virtual void TransferEntities();
      bool CalculateTransfer(Real f_x, Real f_y, std::string& str_engine_id);
      void ScheduleEntityForTransfer(CEntity& c_entity, const std::string& str_engine_id);

      inline virtual bool IsEntityTransferNeeded() const {
         return !m_vecTransferData.empty();
      }

      inline virtual bool IsEntityTransferActive() const {
         return m_bEntityTransferActive;
      }

   private:

      void AddBodiesFromModel(CDynamics3DModel& c_model);
      void AddJointsFromModel(CDynamics3DModel& c_model);

      void RemoveJointsFromModel(CDynamics3DModel& c_model);
      void RemoveBodiesFromModel(CDynamics3DModel& c_model);

   private:

      std::vector<CDynamics3DModel*> m_vecPhysicsModels;
      std::vector<CDynamics3DPlugin*> m_vecPhysicsPlugins;

      /* ARGoS RNG */
      CRandom::CRNG* m_pcRNG;
      CRange<UInt32> m_cRandomSeedRange;
     
      /* Bullet Physics World Data */
      btBroadphaseInterface*                 m_pcBroadphaseInterface;
      btDefaultCollisionConfiguration*       m_pcCollisionConfiguration;
      btCollisionDispatcher*                 m_pcCollisionDispatcher;
      btSequentialImpulseConstraintSolver*   m_pcSolver;
      btDiscreteDynamicsWorld*               m_pcWorld;
      
      /* Dynamics 3D Ground */
      static btStaticPlaneShape              m_cGroundCollisionShape;
      CDynamics3DBody*                       m_pcGround;

      /* Dynamics 3D iterations per pick and iteration length */
      UInt32 m_unIterations;
      Real m_fDeltaT;
      
      /* Transfer entity mechanism data */
      /* @todo create a 3D implementation - at the moment, only prisms with infinite height are supported */
      std::vector<CVector2> m_vecVertices;
      std::vector<SBoundarySegment> m_vecSegments;
      std::vector<SEntityTransferData> m_vecTransferData;
      bool m_bEntityTransferActive;
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
                                                 c_entity);             \
      c_engine.AddPhysicsModel(*pcPhysModel);                           \
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
