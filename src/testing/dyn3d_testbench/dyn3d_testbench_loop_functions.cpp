
/**
 * @file <argos3/testing/dyn3d_testbench/dyn3d_testbench_loop_functions.cpp>
 *
 * @author Michael Allwright <allsey87@gmail.com>
 */


#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/entity/composable_entity.h>

#include "dyn3d_testbench_loop_functions.h"

/****************************************/
/****************************************/

void CDyn3DTestbenchLoopFunctions::Init(TConfigurationNode& t_tree) {
   pcFb = &(dynamic_cast<CComposableEntity&>(m_cSpace.GetEntity("fb1")));
   pcTable = &(dynamic_cast<CComposableEntity&>(m_cSpace.GetEntity("table")));
   pcRoboticArm = &(dynamic_cast<CComposableEntity&>(m_cSpace.GetEntity("robotic-arm")));

   pcRoboticArmBody = &(pcRoboticArm->GetComponent<CEmbodiedEntity>("body"));
   pcTableBody = &(pcTable->GetComponent<CEmbodiedEntity>("body"));
   pcFbBody = &(pcFb->GetComponent<CEmbodiedEntity>("body"));
}


void CDyn3DTestbenchLoopFunctions::PreStep() {
   

   switch(m_cSpace.GetSimulationClock()) {
   case 50:
      // Take the foot-bot off the table and place it on the ground
      pcFbBody->MoveTo(CVector3(1.5,1.5,0), CQuaternion(CRadians::ZERO,CVector3::Z));
      break;

   case 100:
      // Turn the table upside down!
      pcTableBody->MoveTo(CVector3(-1.5,-1.5,0.25), CQuaternion(CRadians::PI,CVector3::X));
      break;

   case 150:
      // Place the robotic arm on top of the side down table
      pcRoboticArmBody->MoveTo(CVector3(-1.5,-1.5,0.5), CQuaternion(CRadians::ZERO,CVector3::Z));
      break;
      
   case 200:
      // Place the foot-bot on top of the robotic arm
      pcFbBody->MoveTo(CVector3(-1.5,-1.5,0), CQuaternion(CRadians::ZERO,CVector3::Z));
      break;

   default:
      break;

   }
   
}

/****************************************/
/****************************************/


void CDyn3DTestbenchLoopFunctions::PostStep() {

}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CDyn3DTestbenchLoopFunctions, "dyn3d_testbench_loop_functions");
