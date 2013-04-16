
/**
 * @file <argos3/testing/dyn3d_testbench/dyn3d_testbench_loop_functions.h>
 *
 * @author Michael Allwright <allsey87@gmail.com>
 */
#ifndef DYN3D_TESTBENCH_LOOP_FUNCTIONS_H
#define DYN3D_TESTBENCH_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>

using namespace argos;

class CDyn3DTestbenchLoopFunctions : public CLoopFunctions {

public:

   virtual void Init(TConfigurationNode& t_tree);
   /**
    * Executes user-defined logic right before a simulation step is executed.
    * The default implementation of this method does nothing.
    * @see PostStep()
    */
   virtual void PreStep();

   /**
    * Executes user-defined logic right after a simulation step is executed.
    * The default implementation of this method does nothing.
    * @see PreStep()
    */
   virtual void PostStep();

private:

   CComposableEntity* pcRoboticArm;
   CComposableEntity* pcTable;
   CComposableEntity* pcFb;
   
   CEmbodiedEntity* pcRoboticArmBody;
   CEmbodiedEntity* pcTableBody;
   CEmbodiedEntity* pcFbBody;
   

};

#endif