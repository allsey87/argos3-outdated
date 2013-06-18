/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_plugin.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "dynamics3d_magnetism_plugin.h"

#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_model.h>
#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_body.h>

namespace argos {

   /****************************************/
   /****************************************/

   void CDynamics3DMagnetismPlugin::Init(TConfigurationNode& t_tree) {

      /*
      CVector3 cMagnetism;
      GetNodeAttribute(t_tree, "force", cMagnetism);
      m_cMagnetism = ARGoSToBullet(cMagnetism); */

      /* parse each decomposition level and the revelevant threshold */
      /* parse the default permitability constant */

   }
   
   /****************************************/
   /****************************************/

   void CDynamics3DMagnetismPlugin::Update(CDynamics3DEngine& c_engine) {
      if(m_bDataStructureInitRequired) {
         InitDataStructures(c_engine);
         m_bDataStructureInitRequired = false;
      }

      fprintf(stderr, "updating plugin CDynamics3DMagnetismPlugin\n");

      fprintf(stderr, "Number of detected bodies = %lu\n", m_vecMagneticBodies.size() );

      //c_engine.
      /** Update the adjacency matrix from pair cache **/

      /** iterate accross adjacency matrix **/

      /** select decomposition level as function of minimum distance **/

      /** use cells from the selected decomposition level to compute forces and torques **/

   }


   void CDynamics3DMagnetismPlugin::InitDataStructures(CDynamics3DEngine& c_engine) {
      
      if(m_ppfMinimumInterbodyDistance != NULL) {
         /* delete the old adjacency matrix */
         for(UInt32 i = 0; i < m_vecMagneticBodies.size(); ++i) {
            delete [] m_ppfMinimumInterbodyDistance[i];
         }
         delete [] m_ppfMinimumInterbodyDistance;
      }
      /* clear the vector of magnetic bodies */
      m_vecMagneticBodies.clear();

      for(CDynamics3DModel::TVector::iterator itModel = c_engine.GetModels().begin();
          itModel != c_engine.GetModels().end();
          ++itModel) {
         for(CDynamics3DBody::TVector::iterator itBody = (*itModel)->GetBodies().begin();
             itBody != (*itModel)->GetBodies().end();
             ++itBody) {

            if((*itBody)->HasAttribute("magnetic") && (*itBody)->GetAttribute("magnetic") == "true") {
               SMagneticBody sMagneticBody(*itBody);
               /*
               for(UInt32 i = 0; i < imax; ++i) {
                  for(UInt32 j = 0; j < jmax; ++j) {
                     for(UInt32 k = 0; k < kmax; ++k) {
                        //sMagneticBody.Cells.push_back(SMagneticCell(i, j, k));
                     }
                  }
               }
               */
               fprintf(stderr, "found magnetic body %s in entity %s\n", (*itBody)->GetId().c_str(), (*itModel)->GetId().c_str());
               
               m_vecMagneticBodies.push_back(sMagneticBody);
            }
         }
      }

      /* allocate storage for the adjacency matrix */
      m_ppfMinimumInterbodyDistance = new Real*[m_vecMagneticBodies.size()];      
      for(UInt32 i = 0; i < m_vecMagneticBodies.size(); ++i) {
         m_ppfMinimumInterbodyDistance[i] = new Real[m_vecMagneticBodies.size()];
      }
      

   }

   /****************************************/
   /****************************************/

   REGISTER_DYN3D_PHYSICS_PLUGIN(CDynamics3DMagnetismPlugin,
                                 "magnetism",
                                 "Michael Allwright [allsey87@gmail.com]",
                                 "1.0",
                                 "A plugin developed for testing the dynamics3d plugin mechanism",
                                 "This plugin applies a specified force to all bodies in the"
                                 "physics engine that it is applied to. The plugin has been "
                                 "developed for testing the dynamics3d plugin mechanism",
                                 "Under development");
}
