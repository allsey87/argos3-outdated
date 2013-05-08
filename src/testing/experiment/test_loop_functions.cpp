
/**
 * @file <argos3/testing/experiment/test_loop_functions.cpp>
 *
 * @author Carlo Pinciroli <ilpincy@gmail.com>
 */
#include "test_loop_functions.h"
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

/****************************************/
/****************************************/

void CTestLoopFunctions::Init(TConfigurationNode& t_tree) {
   LOG << "CTestLoopFunctions init running!\n";
   CFootBotEntity& fb = dynamic_cast<CFootBotEntity&>(m_cSpace.GetEntity("fb"));
   CPositionalEntity& fbledtwoposition = fb.GetComponent<CPositionalEntity>("leds[2]/position");
}

CColor CTestLoopFunctions::GetFloorColor(const CVector2& c_pos_on_floor) {
   if(Sign(c_pos_on_floor.GetX()) == Sign(c_pos_on_floor.GetY()))
      return CColor::GRAY30;
   else
      return CColor::GRAY70;
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CTestLoopFunctions, "test_lf");
