/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics3d/plugins/gravity/dynamics3d_gravity_plugin.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef DYNAMICS3D_GRAVITY_PLUGIN_H
#define DYNAMICS3D_GRAVITY_PLUGIN_H

#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_plugin.h>
#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_body.h>
#include <argos3/core/utility/datatypes/datatypes.h>

namespace argos {
   
   /****************************************/
   /****************************************/
   
   class CDynamics3DGravityPlugin : public CDynamics3DPlugin {
   public:
      CDynamics3DGravityPlugin() :
         m_fGravitationalAcceleration(10.0) {}
      
      ~CDynamics3DGravityPlugin() {}
      
      virtual void Init(TConfigurationNode& t_tree);
      
      virtual void Reset() {}
      virtual void Destroy() {}

      virtual void RegisterModel(CDynamics3DModel& c_model);
      virtual void UnregisterModel(CDynamics3DModel& c_model);
      
      virtual void Update();

   private:
      btScalar m_fGravitationalAcceleration;
      CDynamics3DBody::TVector m_tBodies;
   };
   
   /****************************************/
   /****************************************/
   
}

#endif
