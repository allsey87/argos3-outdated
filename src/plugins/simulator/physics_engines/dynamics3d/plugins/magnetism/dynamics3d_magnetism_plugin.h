/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_plugin.h>
 *
 * @author Haitham Elfaham - <haithamelfaham@gmail.com>
 */

#ifndef DYNAMICS3D_MAGNETISM_PLUGIN_H
#define DYNAMICS3D_MAGNETISM_PLUGIN_H

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

   public:
      struct SMagneticBody {
         SMagneticBody() :
            Parent(NULL),
				Body(NULL),
            Electromagnet(NULL) {}
         SMagneticBody(CDynamics3DModel* pc_parent,
							  CDynamics3DBody* pc_body,
							  CElectromagnetEntity* pc_electromagnet) :
            Parent(pc_parent),
				Body(pc_body),
            Electromagnet(pc_electromagnet) {}

         CDynamics3DModel* Parent;
			CDynamics3DBody* Body;
         CElectromagnetEntity* Electromagnet;

         typedef std::vector<SMagneticBody> TList;
			typedef std::vector<SMagneticBody>::iterator TListIterator;
      };

      Real m_fForceConstant;
      SMagneticBody::TList m_tMagneticBodies;     
   };
   
   /****************************************/
   /****************************************/
   
}

#endif
