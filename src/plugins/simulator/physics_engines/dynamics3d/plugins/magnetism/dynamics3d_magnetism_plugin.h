/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_plugin.h>
 *
 * @author Haitham Elfaham - <haithamelfaham@gmail.com>
 */

#ifndef DYNAMICS3D_MAGNETISM_PLUGIN_H
#define DYNAMICS3D_MAGNETISM_PLUGIN_H

namespace argos {
   
}

#include <vector>

#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_plugin.h>
#include <argos3/plugins/robots/prototype/simulator/entities/electromagnet_equipped_entity.h>
#include <argos3/core/utility/datatypes/datatypes.h>

namespace argos {
   
   /****************************************/
   /****************************************/
   
   class CDynamics3DMagnetismPlugin : public CDynamics3DPlugin {
      
      
   public:
      
      CDynamics3DMagnetismPlugin() {}
      
      ~CDynamics3DMagnetismPlugin() {}
      
      virtual void Init(TConfigurationNode& t_tree);
      
      virtual void Reset() {}
      virtual void Destroy() {}

      virtual void RegisterModel(CDynamics3DModel& c_model);
      virtual void UnregisterModel(CDynamics3DModel& c_model);
      
      virtual void Update();
      
   private:
      
      
      struct SModel {
         CDynamics3DModel* Model;
         CElectromagnetEquippedEntity* Electromagnets;

         std::vector<UInt32> BodyIndices;
         std::vector<UInt32> ElectromagnetIndices;       

         SModel() :
            Model(NULL),
            Electromagnets(NULL) {}

         SModel(CDynamics3DModel* pc_model, CElectromagnetEquippedEntity* pc_electromagnets) :
            Model(pc_model),
            Electromagnets(pc_electromagnets) {}

         typedef std::vector<SModel> TList;
      };
            
      Real m_fForceConstant;
      SModel::TList m_tModels;
      
   };
   
   /****************************************/
   /****************************************/
   
}

#endif
