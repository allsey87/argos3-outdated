/**
 * @file <argos3/plugins/robots/robot/simulator/joint_entity.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "joint_entity.h"

#include <argos3/plugins/robots/robot/simulator/frame_equipped_entity.h>

namespace argos {

   /****************************************/
   /****************************************/

   CJointEntity::CJointEntity(CComposableEntity* pc_parent) :
      CComposableEntity(pc_parent),
      m_bDisableCollisions(false),
      m_pcFrameEquippedEntity(NULL) {}

   /****************************************/
   /****************************************/

   
   CJointEntity::CJointEntity(CComposableEntity* pc_parent,
                              const std::string& str_id,
                              bool b_disable_collisions) :
      CComposableEntity(pc_parent, str_id),
      m_bDisableCollisions(b_disable_collisions) {
      m_pcFrameEquippedEntity = new CFrameEquippedEntity(this, "frames");
   }

   /****************************************/
   /****************************************/

   void CJointEntity::Init(TConfigurationNode& t_tree) {
      try {
         /* Init parent */
         CComposableEntity::Init(t_tree);

         /* check if we are disabling collisions */
         GetNodeAttributeOrDefault(t_tree, "disable_collisions", m_bDisableCollisions, false);

         m_pcFrameEquippedEntity = new CFrameEquippedEntity(this);
         if(NodeExists(t_tree, "frames")) {
            m_pcFrameEquippedEntity->Init(GetNode(t_tree, "frames"));
         }
         AddComponent(*m_pcFrameEquippedEntity);

         /* parse the degrees of freedom if they exist */
         if(NodeExists(t_tree, "axes")) {
            TConfigurationNode& tAxes = GetNode(t_tree, "axes");
            
            TConfigurationNodeIterator itAxis("axis");
            for(itAxis = itAxis.begin(&tAxes);
                itAxis != itAxis.end();
                ++itAxis) {

               std::string strAxisMode, strAxisDirection, strAxisRange;
               GetNodeAttribute(*itAxis, "mode", strAxisMode);
               GetNodeAttribute(*itAxis, "direction", strAxisDirection);
               GetNodeAttribute(*itAxis, "range", strAxisRange);

               bool bAxisUnconstrained = (strAxisRange == "unconstrained");
               bool bAxisHasActuator = NodeExists(*itAxis, "actuator");

               bool bAxisActuatorEnabled = false;
               Real fAxisActuatorForce = 0.0f;
               Real fAxisActuatorTargetVelocity;

               if(bAxisHasActuator) {
                  TConfigurationNode& tActuator = GetNode(*itAxis, "actuator");
                  GetNodeAttribute(tActuator, "enabled", bAxisActuatorEnabled);
                  GetNodeAttribute(tActuator, "force", fAxisActuatorForce);
                  GetNodeAttribute(tActuator, "target_velocity", fAxisActuatorTargetVelocity);
               }
               
               if(strAxisMode == "linear") {
                  CRange<Real> cAxisRange = CRange<Real>();
                  if(!bAxisUnconstrained) {
                     GetNodeAttribute(*itAxis, "range", cAxisRange);
                  }
                  switch(strAxisDirection.c_str()[0]) {
                  case 'x':
                     m_sLinearAxes.m_sX = SAxisParameters<Real>(bAxisUnconstrained,
                                                                cAxisRange,
                                                                bAxisActuatorEnabled,
                                                                fAxisActuatorForce,
                                                                fAxisActuatorTargetVelocity);
                     break;
                  case 'y':
                     m_sLinearAxes.m_sY = SAxisParameters<Real>(bAxisUnconstrained,
                                                                cAxisRange,
                                                                bAxisActuatorEnabled,
                                                                fAxisActuatorForce,
                                                                fAxisActuatorTargetVelocity);
                     break;
                  case 'z':
                     m_sLinearAxes.m_sZ = SAxisParameters<Real>(bAxisUnconstrained,
                                                                cAxisRange,
                                                                bAxisActuatorEnabled,
                                                                fAxisActuatorForce,
                                                                fAxisActuatorTargetVelocity);
                     break;
                  default:
                     THROW_ARGOSEXCEPTION("Error parsing joint axis, axis direction must be one of x, y or z");
                  }
               }
               else if(strAxisMode == "angular") {
                  CRange<CDegrees> cAxisRangeDegrees = CRange<CDegrees>();
                  CRange<CRadians> cAxisRangeRadians = CRange<CRadians>();
                  if(!bAxisUnconstrained) {
                     GetNodeAttribute(*itAxis, "range", cAxisRangeDegrees);
                     cAxisRangeRadians.Set(ToRadians(cAxisRangeDegrees.GetMin()),
                                           ToRadians(cAxisRangeDegrees.GetMax()));
                  }
                  switch(strAxisDirection.c_str()[0]) {
                  case 'x':
                     m_sAngularAxes.m_sX = SAxisParameters<CRadians>(bAxisUnconstrained,
                                                                     cAxisRangeRadians,
                                                                     bAxisActuatorEnabled,
                                                                     fAxisActuatorForce,
                                                                     fAxisActuatorTargetVelocity);
                     break;
                  case 'y':
                     m_sAngularAxes.m_sY = SAxisParameters<CRadians>(bAxisUnconstrained,
                                                                     cAxisRangeRadians,
                                                                     bAxisActuatorEnabled,
                                                                     fAxisActuatorForce,
                                                                     fAxisActuatorTargetVelocity);
                     break;
                  case 'z':
                     m_sAngularAxes.m_sZ = SAxisParameters<CRadians>(bAxisUnconstrained,
                                                                     cAxisRangeRadians,
                                                                     bAxisActuatorEnabled,
                                                                     fAxisActuatorForce,
                                                                     fAxisActuatorTargetVelocity);
                     break;
                  default:
                     THROW_ARGOSEXCEPTION("Error parsing joint axis, axis direction must be one of x, y or z");
                  }
               }
               else {
                  THROW_ARGOSEXCEPTION("Error parsing joint axis, axis mode must be either linear or angular");
               }
            }
         }
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Error while initializing joint entity", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CJointEntity::Reset() {
      CComposableEntity::Reset();
      m_pcFrameEquippedEntity->Reset();
   }

   /****************************************/
   /****************************************/

   void CJointEntity::Destroy() {
      //m_pcFrameEquippedEntity->Destroy();      
      CComposableEntity::Destroy();
   }

   /****************************************/
   /****************************************/

   void CJointEntity::UpdateComponents() {}

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_SPACE_OPERATIONS_ON_COMPOSABLE(CJointEntity);

   /****************************************/
   /****************************************/

}
