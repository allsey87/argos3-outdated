
/**
 * @file <argos3/testing/dyn3d_testbench/dyn3d_testbench_repeatability_loop_functions.cpp>
 *
 * @author Michael Allwright <allsey87@gmail.com>
 */


#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/core/utility/math/quaternion.h>

#include "dyn3d_testbench_repeatability_loop_functions.h"

/****************************************/
/****************************************/

void CDyn3DTestbenchRepeatabilityLoopFunctions::Init(TConfigurationNode& t_tree) {
   pcTestBox = &(dynamic_cast<CComposableEntity&>(GetSpace().GetEntity("test_box")));
   pcTestCylinder = &(dynamic_cast<CComposableEntity&>(GetSpace().GetEntity("test_cylinder")));

   pcTestBoxBody = &(pcTestBox->GetComponent<CEmbodiedEntity>("body"));
   pcTestCylinderBody = &(pcTestCylinder->GetComponent<CEmbodiedEntity>("body"));
}


void CDyn3DTestbenchRepeatabilityLoopFunctions::PreStep() {
   CRadians c_angle;
   CVector3 c_axis;
   
   pcTestBoxBody->GetOrientation().ToAngleAxis(c_angle, c_axis);
   
   fprintf(stderr, 
           "test_box: position = [%.3f, %.3f, %.3f], orientation axis = [%.3f, %.3f, %.3f], orientation_angle = [%.3f]\n",
            pcTestBoxBody->GetPosition().GetX(), pcTestBoxBody->GetPosition().GetY(), pcTestBoxBody->GetPosition().GetZ(),
            c_axis.GetX(), c_axis.GetY(), c_axis.GetZ(), c_angle.GetValue());             
            
   pcTestCylinderBody->GetOrientation().ToAngleAxis(c_angle, c_axis);

   fprintf(stderr, 
        "test_cylinder: position = [%.3f, %.3f, %.3f], orientation axis = [%.3f, %.3f, %.3f], orientation_angle = [%.3f]\n",
         pcTestCylinderBody->GetPosition().GetX(), pcTestCylinderBody->GetPosition().GetY(), pcTestCylinderBody->GetPosition().GetZ(),
         c_axis.GetX(), c_axis.GetY(), c_axis.GetZ(), c_angle.GetValue());             
}

/****************************************/
/****************************************/


void CDyn3DTestbenchRepeatabilityLoopFunctions::PostStep() {

}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CDyn3DTestbenchRepeatabilityLoopFunctions, "dyn3d_testbench_repeatability_loop_functions");
