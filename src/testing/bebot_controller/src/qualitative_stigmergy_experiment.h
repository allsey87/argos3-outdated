#ifndef QUALITATIVE_EXPERIMENT_H
#define QUALITATIVE_EXPERIMENT_H

#include "state.h"
#include "block_demo.h"
#include "pid_controller.h"

#include <iostream>

#define IMAGE_SENSOR_WIDTH 640.0
#define IMAGE_SENSOR_HALF_WIDTH (IMAGE_SENSOR_WIDTH / 2.0)
#define IMAGE_SENSOR_HEIGHT 360.0
#define IMAGE_SENSOR_HALF_HEIGHT (IMAGE_SENSOR_HEIGHT / 2.0)

#define LIFT_ACTUATOR_MAX_HEIGHT 135
#define LIFT_ACTUATOR_MIN_HEIGHT 3
#define LIFT_ACTUATOR_INC_HEIGHT 15
#define LIFT_ACTUATOR_BLOCK_HEIGHT 55
#define LIFT_ACTUATOR_BLOCK_PREATTACH_OFFSET 3

#define RF_UN_BLOCK_DETECT_THRES 2250
#define RF_LR_BLOCK_DETECT_THRES 3500
#define RF_FLR_BLOCK_DETECT_THRES 2500
#define RF_FLR_BLOCK_CONTACT_THRES 2675
#define RF_FCNR_ROBOT_DETECT_THRES 1500

#define OBSERVE_BLOCK_X_TARGET 0.000
#define OBSERVE_BLOCK_Z_TARGET 0.275
#define OBSERVE_BLOCK_XZ_THRES 0.050

#define PREAPPROACH_BLOCK_X_TARGET 0.000
#define PREAPPROACH_BLOCK_Z_TARGET 0.325
#define PREAPPROACH_BLOCK_XZ_THRES 0.025

#define PREPLACEMENT_BLOCK_X_TARGET 0.000
#define PREPLACEMENT_BLOCK_X_THRES 0.010

#define TAG_OFFSET_TARGET 0.700

#define APPROACH_BLOCK_X_FAIL_THRES 0.025

#define NEAR_APPROACH_TIMEOUT std::chrono::milliseconds(15000)
#define REVERSE_TIMEOUT_SHORT std::chrono::milliseconds(7500)
#define REVERSE_TIMEOUT_LONG std::chrono::milliseconds(10000)

#define BLOCK_TYPE_OFF "0"
#define BLOCK_TYPE_Q1  "1"
#define BLOCK_TYPE_Q2  "2"
#define BLOCK_TYPE_Q3  "3"
#define BLOCK_TYPE_Q4  "4"

#define BLOCK_SIDE_LENGTH 0.055
#define BLOCK_HALF_SIDE_LENGTH (BLOCK_SIDE_LENGTH / 2.0)

#define BASE_VELOCITY 30.0
#define BASE_XZ_GAIN 7.5

// when lift actuator is at zero
#define CAMERA_VERTICAL_OFFSET_MM 105

ELedState GetBlockLedState(const SBlock& s_block) {
   std::map<ELedState, unsigned int> mapLedCounts = {
      std::make_pair(ELedState::OFF, GetLedCount(s_block, {ELedState::OFF})),
      std::make_pair(ELedState::Q1, GetLedCount(s_block, {ELedState::Q1})),
      std::make_pair(ELedState::Q2, GetLedCount(s_block, {ELedState::Q2})),
      std::make_pair(ELedState::Q3, GetLedCount(s_block, {ELedState::Q3})),
      std::make_pair(ELedState::Q4, GetLedCount(s_block, {ELedState::Q4})),
   };
   auto itMaxLedCount = std::max_element(std::begin(mapLedCounts), std::end(mapLedCounts), 
      [] (const std::pair<const ELedState, unsigned int>& c_pair_lhs,
          const std::pair<const ELedState, unsigned int>& c_pair_rhs) {
      return (c_pair_lhs.second < c_pair_rhs.second);
   });
   return ((itMaxLedCount != std::end(mapLedCounts)) ? itMaxLedCount->first : ELedState::OFF);
}


struct SGlobalData : CState::SData {
   /************************************************************/
   /*                       Constructor                        */
   /************************************************************/
   SGlobalData(CBlockDemo::SSensorData* ps_sensor_data,
               CBlockDemo::SActuatorData* ps_actuator_data) :
      Sensors(ps_sensor_data),
      Actuators(ps_actuator_data),
      TagApproachController(4.250,0.100,1.375,0.500),
      TargetInRange(false),
      TrackedStructureId(0u),
      TrackedTargetId(0u),
      NextLedStateToAssign(ELedState::Q3),
      RandomNumberGenerator(argos::CRandom::CreateRNG("argos")) {}
   
   /************************************************************/
   /*               Sensor and actuator access                 */
   /************************************************************/
   CBlockDemo::SSensorData* Sensors;
   CBlockDemo::SActuatorData* Actuators;
   /* controllers */
   CPIDController TagApproachController;

   /************************************************************/
   /*               Shared data for all states                 */
   /************************************************************/
   bool TargetInRange;
   unsigned int TrackedStructureId;
   unsigned int TrackedTargetId;

   struct {
      argos::CVector3 Translation;
      argos::CQuaternion Rotation;
   } TrackedTargetLastObservation;

   ELedState NextLedStateToAssign;

   argos::CRandom::CRNG* RandomNumberGenerator;

   std::chrono::time_point<std::chrono::steady_clock> ElectromagnetSwitchOnTime;
   std::chrono::time_point<std::chrono::steady_clock> NearApproachStartTime;
   std::chrono::time_point<std::chrono::steady_clock> ReverseToFindTargetStartTime;

   /************************************************************/
   /*            Common functions for all states               */
   /************************************************************/
   bool SetTargetInRange() {
      TargetInRange = true;
      return true;
   }

   bool ClearTargetInRange() {
      TargetInRange = false;
      return true;
   }

   bool IsTargetInRange() {
      return TargetInRange;
   }

   bool IsTargetLost() {
      auto itTarget = FindTrackedTarget(TrackedTargetId, Sensors->ImageSensor.Detections.Targets);
      if(itTarget == std::end(Sensors->ImageSensor.Detections.Targets)) {
         return true;
      }
      return false;
   }

   bool IsNextTargetAcquired() {
      std::vector<STarget::TListIterator> vecDetectedTargets;
      for(STarget::TListIterator itDetectedTarget = std::begin(Sensors->ImageSensor.Detections.Targets);
          itDetectedTarget != std::end(Sensors->ImageSensor.Detections.Targets);
          itDetectedTarget++) {
         if(itDetectedTarget->Id > TrackedTargetId) {
            vecDetectedTargets.push_back(itDetectedTarget);
         }
      }
      std::sort(std::begin(vecDetectedTargets), 
                std::end(vecDetectedTargets), 
                [] (const STarget::TListIterator& it_target_lhs,
                    const STarget::TListIterator& it_target_rhs) {
         return (it_target_lhs->Id < it_target_rhs->Id);
      });
      std::vector<STarget::TListIterator>::iterator itAcquiredTarget = 
         std::find_if(std::begin(vecDetectedTargets), 
                      std::end(vecDetectedTargets), 
                      [] (const STarget::TListIterator& it_target) {
         /* select the first non-Q4 target */
         return (GetBlockLedState(it_target->Observations.front()) != ELedState::Q4);
      });
      if(itAcquiredTarget != std::end(vecDetectedTargets)) {
         TrackedTargetId = (*itAcquiredTarget)->Id;
         return true;
      }
      return false;
   }

   double TrackBlockViaLiftActuatorHeight(const SBlock& s_block,
                                          std::function<const STag::TCoordinate&(const STag&)> fn_get_coordinate = GetTagCenter,
                                          uint8_t un_min_position = LIFT_ACTUATOR_MIN_HEIGHT,
                                          uint8_t un_max_position = LIFT_ACTUATOR_MAX_HEIGHT) {
      double fEndEffectorPos = Sensors->ManipulatorModule.LiftActuator.EndEffector.Position;
      double fLiftActuatorHeightError = (IMAGE_SENSOR_HALF_HEIGHT - fn_get_coordinate(s_block.Tags[0]).second) / IMAGE_SENSOR_HALF_HEIGHT;
      fEndEffectorPos += (fLiftActuatorHeightError * LIFT_ACTUATOR_INC_HEIGHT);
      uint8_t unEndEffectorPos =
         (fEndEffectorPos > un_max_position) ? un_max_position :
         (fEndEffectorPos < un_min_position) ? un_min_position : static_cast<uint8_t>(fEndEffectorPos);
      Actuators->ManipulatorModule.LiftActuator.Position.Value = unEndEffectorPos;
      Actuators->ManipulatorModule.LiftActuator.Position.UpdateReq = true;
      return fLiftActuatorHeightError;
   }

   void SetVelocity(double f_left, double f_right) {
      Actuators->DifferentialDriveSystem.Left.Velocity = std::round(f_left);
      Actuators->DifferentialDriveSystem.Right.Velocity = std::round(f_right);
      Actuators->DifferentialDriveSystem.Left.UpdateReq = true;
      Actuators->DifferentialDriveSystem.Right.UpdateReq = true;
   }
};


/**************** functions for statistics ****************/
double GetMedian(const std::list<uint16_t>& lst_data) { 
   std::vector<uint16_t> vecData(std::begin(lst_data), std::end(lst_data));
   size_t n = vecData.size() / 2;
   std::nth_element(std::begin(vecData), std::begin(vecData) + n, std::end(vecData));
   uint16_t vn = vecData[n];
   if(vecData.size() % 2 == 1) {
      return 1.0 * vn;
   }
   else {
      std::nth_element(std::begin(vecData), std::begin(vecData) + (n - 1), std::end(vecData));
      return 0.5 * (vn + vecData[n - 1]);
   }
}

argos::CVector3 GetAdjBlockTranslation(const SBlock& s_block) {
   argos::CRadians cYZAngle = -argos::ATan2(s_block.Translation.GetY(), s_block.Translation.GetZ()) +
                               argos::CRadians::PI_OVER_FOUR;
   double fYZDistance = std::hypot(s_block.Translation.GetY(), s_block.Translation.GetZ());
   return argos::CVector3(fYZDistance * argos::Sin(cYZAngle),
                          s_block.Translation.GetX(),
                          fYZDistance * argos::Cos(cYZAngle));
}

/**************** functions for qualitative experiment ****************/

unsigned int GetBlockLevel(const SBlock& s_block, unsigned int un_lift_actuator_position) {
   double fBlockPositionOffset = GetAdjBlockTranslation(s_block).GetZ();
   // TODO stop using millimeters, everything should be in SI units and stored with double-precision
   double fBlockPosition = ((CAMERA_VERTICAL_OFFSET_MM + un_lift_actuator_position) / 1000.0) - fBlockPositionOffset;
   double fBlockLevel = std::round(std::abs(fBlockPosition - BLOCK_HALF_SIDE_LENGTH) / BLOCK_SIDE_LENGTH);
   return static_cast<unsigned int>(fBlockLevel);
}

STarget::TConstListIterator FindQualitativeTarget(const STarget::TList& t_list) {
   for(STarget::TConstListIterator it_target = std::begin(t_list); it_target != std::end(t_list); it_target++) {
      if(GetBlockLedState(it_target->Observations.front()) == ELedState::Q1) {
         return it_target;
      }
   }
   return std::end(t_list);
}


/************************************************************/
/*               Common state definitions                   */
/************************************************************/

class CStatePulseElectromagnets : public CState {
public:
   CStatePulseElectromagnets(const std::string& str_id, CState* pc_parent, const std::chrono::milliseconds& t_duration, CBlockDemo::EGripperFieldMode e_field_mode) :
      CState(str_id, pc_parent, nullptr, CState::TVector {
         AddState<CState>("init_precharge", [this] {
            GetBase().GetData<SGlobalData>().Actuators->ManipulatorModule.EndEffector.FieldMode = CBlockDemo::EGripperFieldMode::DISABLED;
            GetBase().GetData<SGlobalData>().Actuators->ManipulatorModule.EndEffector.UpdateReq = true;
         }),
         AddState<CState>("wait_for_precharge"),
         AddState<CState>("switch_field_on", [this, e_field_mode] {
            GetBase().GetData<SGlobalData>().Actuators->ManipulatorModule.EndEffector.FieldMode = e_field_mode;
            GetBase().GetData<SGlobalData>().Actuators->ManipulatorModule.EndEffector.UpdateReq = true;
            GetBase().GetData<SGlobalData>().ElectromagnetSwitchOnTime = GetBase().GetData<SGlobalData>().Sensors->Clock.Time;
         }),
         AddState<CState>("wait_for_duration"),
         AddState<CState>("switch_field_off", [this] {
            GetBase().GetData<SGlobalData>().Actuators->ManipulatorModule.EndEffector.FieldMode = CBlockDemo::EGripperFieldMode::DISABLED;
            GetBase().GetData<SGlobalData>().Actuators->ManipulatorModule.EndEffector.UpdateReq = true;
         }),
      }) {
      AddTransition("init_precharge","wait_for_precharge");
      AddTransition("wait_for_precharge","switch_field_on", [this] {
         auto tMinMaxPair = std::minmax_element(std::begin(GetBase().GetData<SGlobalData>().Sensors->ManipulatorModule.LiftActuator.Electromagnets.Charge),
                                                std::end(GetBase().GetData<SGlobalData>().Sensors->ManipulatorModule.LiftActuator.Electromagnets.Charge));
         return *(tMinMaxPair.first) == *(tMinMaxPair.second);
      });
      AddTransition("switch_field_on","wait_for_duration");
      AddTransition("wait_for_duration", "switch_field_off", [this, t_duration] {
         return (GetBase().GetData<SGlobalData>().ElectromagnetSwitchOnTime + t_duration) < GetBase().GetData<SGlobalData>().Sensors->Clock.Time;
      });
      AddExitTransition("switch_field_off");
   }
};

class CStateSetLiftActuatorPosition : public CState {
public:
   CStateSetLiftActuatorPosition(const std::string& str_id, CState* pc_parent, uint8_t un_position) :
      CState(str_id, pc_parent, nullptr, CState::TVector {
         AddState<CState>("set_position", [this, un_position] {
            GetBase().GetData<SGlobalData>().Actuators->ManipulatorModule.LiftActuator.Position.Value = un_position;
            GetBase().GetData<SGlobalData>().Actuators->ManipulatorModule.LiftActuator.Position.UpdateReq = true;
         }),
         AddState<CState>("wait_for_lift_actuator"),
      }) {
      AddTransition("set_position","wait_for_lift_actuator");
      AddExitTransition("wait_for_lift_actuator", [this] {
         return (GetBase().GetData<SGlobalData>().Sensors->ManipulatorModule.LiftActuator.State ==
                 CBlockDemo::ELiftActuatorSystemState::INACTIVE);
      });
   }
};

class CStateAttachBlock : public CState {
public:
   CStateAttachBlock(const std::string& str_id, CState* pc_parent) :
      CState(str_id, pc_parent, nullptr, {
         AddState<CStateSetLiftActuatorPosition>("init_lift_actuator_position", LIFT_ACTUATOR_MIN_HEIGHT + LIFT_ACTUATOR_BLOCK_PREATTACH_OFFSET),
         AddState<CStatePulseElectromagnets>("generate_pre_alignment_pulse", std::chrono::milliseconds(500), CBlockDemo::EGripperFieldMode::CONSTRUCTIVE),
         AddState<CStateSetLiftActuatorPosition>("lower_lift_actuator", LIFT_ACTUATOR_MIN_HEIGHT),
         AddState<CStatePulseElectromagnets>("generate_attachment_pulse", std::chrono::milliseconds(1000), CBlockDemo::EGripperFieldMode::CONSTRUCTIVE),
      }) {
         AddTransition("init_lift_actuator_position","generate_pre_alignment_pulse");
         AddTransition("generate_pre_alignment_pulse", "lower_lift_actuator");
         AddTransition("lower_lift_actuator","generate_attachment_pulse");
         AddExitTransition("generate_attachment_pulse");
      }
};

class CStateSetLedColors : public CState {
public:
   CStateSetLedColors(const std::string& str_id, CState* pc_parent, CBlockDemo::EColor e_new_color) :
      CState(str_id, pc_parent, [this, e_new_color] {
         for(CBlockDemo::EColor& e_color : GetBase().GetData<SGlobalData>().Actuators->LEDDeck.Color)
            e_color = e_new_color;
         for(bool& b_update : GetBase().GetData<SGlobalData>().Actuators->LEDDeck.UpdateReq)
            b_update = true;
   }) {}
};

class CStateSendNFCMessage : public CState {
public:
   CStateSendNFCMessage(const std::string& str_id, CState* pc_parent, const std::string& str_data) :
      CState(str_id, pc_parent, [this, str_data] {
         GetBase().GetData<SGlobalData>().Actuators->ManipulatorModule.NFCInterface.OutboundMessage = str_data;
         GetBase().GetData<SGlobalData>().Actuators->ManipulatorModule.NFCInterface.UpdateReq = true;
   }) {}
};

class CStateSetVelocity : public CState {
public:
   CStateSetVelocity(const std::string& str_id, CState* pc_parent, double f_left, double f_right) :
      CState(str_id, pc_parent, [this, f_left, f_right] {
         /* apply the approach velocity */
         GetBase().GetData<SGlobalData>().SetVelocity(f_left, f_right);
      }) {}
};

class CStateMoveToTargetXZ : public CState {
public:
   CStateMoveToTargetXZ(const std::string& str_id, CState* pc_parent, double f_x_target, double f_z_target, bool b_track_via_lift_actuator) :
      CState(str_id, pc_parent, [this, f_x_target, f_z_target, b_track_via_lift_actuator] {
         /* default velocities, overwritten if target is detected */
         double fLeft = 0.000, fRight = 0.000;
         /* select tracked target */
         auto itTarget = FindTrackedTarget(GetBase().GetData<SGlobalData>().TrackedTargetId, GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets);
         if(itTarget != std::end(GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            /* calculate the approach velocity */
            double fBlockXOffset = s_block.Translation.GetX() - f_x_target;
            double fBlockZOffset = s_block.Translation.GetZ() - f_z_target;
            fLeft = BASE_VELOCITY * (fBlockXOffset + fBlockZOffset) * BASE_XZ_GAIN;
            fRight = BASE_VELOCITY * (-fBlockXOffset + fBlockZOffset) * BASE_XZ_GAIN;
            /* track target via the lift actuator if enabled */
            if(b_track_via_lift_actuator) {
               GetBase().GetData<SGlobalData>().TrackBlockViaLiftActuatorHeight(s_block, FindTagCornerFurthestToTheTop, LIFT_ACTUATOR_MIN_HEIGHT + LIFT_ACTUATOR_BLOCK_PREATTACH_OFFSET);
               /* adjust speed if the tag is falling out of the frame */
               double fTagOffsetTop =
                  std::abs(FindTagCornerFurthestToTheTop(s_block.Tags[0]).second - IMAGE_SENSOR_HALF_HEIGHT) / IMAGE_SENSOR_HALF_HEIGHT;
               double fTagOffsetBottom =
                  std::abs(FindTagCornerFurthestToTheBottom(s_block.Tags[0]).second - IMAGE_SENSOR_HALF_HEIGHT) / IMAGE_SENSOR_HALF_HEIGHT;
               fLeft *= std::max(1.000 - std::max(fTagOffsetTop, fTagOffsetBottom), 0.250);
               fRight *= std::max(1.000 - std::max(fTagOffsetTop, fTagOffsetBottom), 0.250);
            }
         }
         /* apply the approach velocity */
         GetBase().GetData<SGlobalData>().SetVelocity(fLeft, fRight);
      }) {}
};

class CStateMoveToTargetX : public CState {
public:
   CStateMoveToTargetX(const std::string& str_id, CState* pc_parent, double f_x_target, bool b_track_via_lift_actuator) :
      CState(str_id, pc_parent, [this, f_x_target, b_track_via_lift_actuator] {
         /* default velocities, overwritten if target is detected */
         double fLeft = 0.000, fRight = 0.000;
         /* select tracked target */
         auto itTarget = FindTrackedTarget(GetBase().GetData<SGlobalData>().TrackedTargetId, GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets);
         if(itTarget != std::end(GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            /* calculate the approach velocity */
            double fBlockXOffset = s_block.Translation.GetX() - f_x_target;
            fLeft = BASE_VELOCITY * (fBlockXOffset) * BASE_XZ_GAIN;
            fRight = BASE_VELOCITY * (-fBlockXOffset) * BASE_XZ_GAIN;
            /* track target via the lift actuator if enabled */
            if(b_track_via_lift_actuator) {
               GetBase().GetData<SGlobalData>().TrackBlockViaLiftActuatorHeight(s_block, GetTagCenter, LIFT_ACTUATOR_MIN_HEIGHT + LIFT_ACTUATOR_BLOCK_PREATTACH_OFFSET);
               /* adjust speed if the tag is falling out of the frame */
               double fTagOffsetTop =
                  std::abs(FindTagCornerFurthestToTheTop(s_block.Tags[0]).second - IMAGE_SENSOR_HALF_HEIGHT) / IMAGE_SENSOR_HALF_HEIGHT;
               double fTagOffsetBottom =
                  std::abs(FindTagCornerFurthestToTheBottom(s_block.Tags[0]).second - IMAGE_SENSOR_HALF_HEIGHT) / IMAGE_SENSOR_HALF_HEIGHT;
               fLeft *= std::max(1.000 - std::max(fTagOffsetTop, fTagOffsetBottom), 0.250);
               fRight *= std::max(1.000 - std::max(fTagOffsetTop, fTagOffsetBottom), 0.250);
            }
         }
         /* apply the approach velocity */
         GetBase().GetData<SGlobalData>().SetVelocity(fLeft, fRight);
      }) {}
};

class CStateMoveToTargetZ : public CState {
public:
   CStateMoveToTargetZ(const std::string& str_id, CState* pc_parent, double f_z_target, bool b_track_via_lift_actuator) :
      CState(str_id, pc_parent, [this, f_z_target, b_track_via_lift_actuator] {
         /* default velocities, overwritten if target is detected */
         double fLeft = 0.000, fRight = 0.000;
         /* select tracked target */
         auto itTarget = FindTrackedTarget(GetBase().GetData<SGlobalData>().TrackedTargetId, GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets);
         if(itTarget != std::end(GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            /* calculate the approach velocity */
            double fBlockZOffset = s_block.Translation.GetZ() - f_z_target;
            fLeft = BASE_VELOCITY * (fBlockZOffset) * BASE_XZ_GAIN;
            fRight = BASE_VELOCITY * (fBlockZOffset) * BASE_XZ_GAIN;
            /* track target via the lift actuator if enabled */
            if(b_track_via_lift_actuator) {
               GetBase().GetData<SGlobalData>().TrackBlockViaLiftActuatorHeight(s_block, GetTagCenter, LIFT_ACTUATOR_MIN_HEIGHT + LIFT_ACTUATOR_BLOCK_PREATTACH_OFFSET);
               /* adjust speed if the tag is falling out of the frame */
               double fTagOffsetTop =
                  std::abs(FindTagCornerFurthestToTheTop(s_block.Tags[0]).second - IMAGE_SENSOR_HALF_HEIGHT) / IMAGE_SENSOR_HALF_HEIGHT;
               double fTagOffsetBottom =
                  std::abs(FindTagCornerFurthestToTheBottom(s_block.Tags[0]).second - IMAGE_SENSOR_HALF_HEIGHT) / IMAGE_SENSOR_HALF_HEIGHT;
               fLeft *= std::max(1.000 - std::max(fTagOffsetTop, fTagOffsetBottom), 0.250);
               fRight *= std::max(1.000 - std::max(fTagOffsetTop, fTagOffsetBottom), 0.250);
            }
         }
         /* apply the approach velocity */
         GetBase().GetData<SGlobalData>().SetVelocity(fLeft, fRight);
      }) {}
};

class CStateAlignWithTagOffset : public CState {
public:
   CStateAlignWithTagOffset(const std::string& str_id, CState* pc_parent, double f_tag_offset_target,
                            std::function<const STag::TCoordinate&(const STag&)> fn_get_coordinate) :
      CState(str_id, pc_parent, [this, f_tag_offset_target, fn_get_coordinate] {
            /* default velocities, overwritten if target is detected */
            double fLeft = 0.000, fRight = 0.000;
            /* select tracked target */
            auto itTarget = FindTrackedTarget(GetBase().GetData<SGlobalData>().TrackedTargetId, GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets);
            if(itTarget != std::end(GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets)) {
               const SBlock& s_block = itTarget->Observations.front();
               double fTagOffset = (fn_get_coordinate(s_block.Tags[0]).first - IMAGE_SENSOR_HALF_WIDTH) / IMAGE_SENSOR_HALF_WIDTH;
               fLeft  = (fTagOffset - f_tag_offset_target) * BASE_VELOCITY;
               fRight = (f_tag_offset_target - fTagOffset) * BASE_VELOCITY;
            }
            GetBase().GetData<SGlobalData>().SetVelocity(fLeft, fRight);
      }) {}
};

class CStateApproachTarget : public CState {
public:
   CStateApproachTarget(const std::string& str_id, CState* pc_parent, double f_lift_actuator_min_height, double f_tag_offset_target, 
                        std::function<const STag::TCoordinate&(const STag&)> fn_get_coordinate) :
      CState(str_id, pc_parent, [this, f_lift_actuator_min_height, f_tag_offset_target, fn_get_coordinate] {
         // default velocities, overwritten if target is detected
         double fLeft = 0.000, fRight = 0.000;
         // select tracked target
         auto itTarget = FindTrackedTarget(GetBase().GetData<SGlobalData>().TrackedTargetId, GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets);
         if(itTarget != std::end(GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            /* update the last observation data */
            GetBase().GetData<SGlobalData>().TrackedTargetLastObservation.Rotation = s_block.Rotation;
            GetBase().GetData<SGlobalData>().TrackedTargetLastObservation.Translation = s_block.Translation;
            /* track the target by lowering the lift actuator position */
            GetBase().GetData<SGlobalData>().TrackBlockViaLiftActuatorHeight(s_block, GetTagCenter,
                                            f_lift_actuator_min_height,
                                            GetBase().GetData<SGlobalData>().Actuators->ManipulatorModule.LiftActuator.Position.Value);
            /* calculate the steering variable */
            double fTagOffset = (fn_get_coordinate(s_block.Tags[0]).first - IMAGE_SENSOR_HALF_WIDTH) / IMAGE_SENSOR_HALF_WIDTH;
            double fOutput = GetBase().GetData<SGlobalData>().TagApproachController.Step(fTagOffset, f_tag_offset_target, GetBase().GetData<SGlobalData>().Sensors->Clock.Time);
            /* saturate the steering variable between 0 and 1 */
            fOutput = (fOutput > 0.750) ? 0.750 : ((fOutput < -0.750) ? -0.750 : fOutput);
            /* calculate the approach velocities */
            fRight = (1.000 + fOutput) * BASE_VELOCITY;
            fLeft  = (1.000 - fOutput) * BASE_VELOCITY;
            /* scale the speed by the lift actuator height error */
            double fTagOffsetTop =
               std::abs(FindTagCornerFurthestToTheTop(s_block.Tags[0]).second - IMAGE_SENSOR_HALF_HEIGHT) / IMAGE_SENSOR_HALF_HEIGHT;
            double fTagOffsetBottom =
               std::abs(FindTagCornerFurthestToTheBottom(s_block.Tags[0]).second - IMAGE_SENSOR_HALF_HEIGHT) / IMAGE_SENSOR_HALF_HEIGHT;
            fLeft *= std::max(1.000 - std::max(fTagOffsetTop, fTagOffsetBottom), 0.250);
            fRight *= std::max(1.000 - std::max(fTagOffsetTop, fTagOffsetBottom), 0.250);
         }
         /* apply the approach velocity */
         GetBase().GetData<SGlobalData>().SetVelocity(fLeft, fRight);
      }) {}
};

class CStateApproachTargetFar : public CState {
public:
   CStateApproachTargetFar(const std::string& str_id, CState* pc_parent, double f_tag_offset_target,
                           std::function<const STag::TCoordinate&(const STag&)> fn_get_coordinate) :
      CState(str_id, pc_parent, nullptr, CState::TVector {
         /*** states (std::vector<CState> initializer list) ***/
         AddState<CStateAlignWithTagOffset>("align_with_tag_offset", f_tag_offset_target, fn_get_coordinate),
         AddState<CStateApproachTarget>("approach_target", LIFT_ACTUATOR_MIN_HEIGHT + LIFT_ACTUATOR_BLOCK_PREATTACH_OFFSET, f_tag_offset_target, fn_get_coordinate),
         // failure states
         AddState<CState>("adjust_lift_actuator_height", [this] {
            if(GetBase().GetData<SGlobalData>().Actuators->ManipulatorModule.LiftActuator.Position.Value < (LIFT_ACTUATOR_MAX_HEIGHT - LIFT_ACTUATOR_BLOCK_PREATTACH_OFFSET)) {
               GetBase().GetData<SGlobalData>().Actuators->ManipulatorModule.LiftActuator.Position.Value += LIFT_ACTUATOR_BLOCK_PREATTACH_OFFSET;
               GetBase().GetData<SGlobalData>().Actuators->ManipulatorModule.LiftActuator.Position.UpdateReq = true;
            }
         }),
         AddState<CStateSetVelocity>("set_reverse_velocity", -0.250 * BASE_VELOCITY, -0.250 * BASE_VELOCITY),
         AddState<CState>("wait_for_target_or_timeout"),
      }) {
         /*** transitions (constructor body) ***/
         AddTransition("align_with_tag_offset", "approach_target", [this, f_tag_offset_target, fn_get_coordinate] {
            auto itTarget = FindTrackedTarget(GetBase().GetData<SGlobalData>().TrackedTargetId, GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets);
            if(itTarget != std::end(GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets)) {
               const SBlock& s_block = itTarget->Observations.front();
               double fTagOffset = (fn_get_coordinate(s_block.Tags[0]).first - IMAGE_SENSOR_HALF_WIDTH) / IMAGE_SENSOR_HALF_WIDTH;
               if(std::abs(fTagOffset - f_tag_offset_target) < 0.1) {         
                  GetBase().GetData<SGlobalData>().TagApproachController.Reset();
                  return true;
               }   
            }
            return false;
         });
         AddExitTransition("approach_target", [this] {
            bool bTargetLost = GetBase().GetData<SGlobalData>().IsTargetLost();
            bool bLiftActuatorAtBottom = 
               (GetBase().GetData<SGlobalData>().Actuators->ManipulatorModule.LiftActuator.Position.Value <= (LIFT_ACTUATOR_MIN_HEIGHT + LIFT_ACTUATOR_BLOCK_PREATTACH_OFFSET));
            bool bLastObservationInRange =
               (std::abs(GetBase().GetData<SGlobalData>().TrackedTargetLastObservation.Translation.GetX()) <= APPROACH_BLOCK_X_FAIL_THRES);
            if(bTargetLost && bLiftActuatorAtBottom && bLastObservationInRange) {
               GetBase().GetData<SGlobalData>().SetTargetInRange();
               return true;
            }
            return false;
         });
         AddTransition("align_with_tag_offset", "set_reverse_velocity", [this] {
            return GetBase().GetData<SGlobalData>().IsTargetLost();
         });
         AddTransition("approach_target", "set_reverse_velocity", [this] {
            return GetBase().GetData<SGlobalData>().IsTargetLost();
         });
         AddTransition("set_reverse_velocity", "adjust_lift_actuator_height");
         // back off until target is re-acquired
         AddTransition("adjust_lift_actuator_height", "wait_for_target_or_timeout", [this] {
            // reset timer for the reverse velocity search
            GetBase().GetData<SGlobalData>().ReverseToFindTargetStartTime = GetBase().GetData<SGlobalData>().Sensors->Clock.Time;
            return true;
         });
         // try again
         AddTransition("wait_for_target_or_timeout", "align_with_tag_offset", [this] {
            return GetBase().GetData<SGlobalData>().IsNextTargetAcquired();
         });
         // timer has expired
         AddExitTransition("wait_for_target_or_timeout", [this] {
            if(GetBase().GetData<SGlobalData>().ReverseToFindTargetStartTime + REVERSE_TIMEOUT_SHORT < GetBase().GetData<SGlobalData>().Sensors->Clock.Time) {
               GetBase().GetData<SGlobalData>().ClearTargetInRange();
               return true;
            }
            return false;
         });
      }
};

class CStateApproachTargetNear : public CState {
public:
   CStateApproachTargetNear(const std::string& str_id, CState* pc_parent) :
      CState(str_id, pc_parent, nullptr, CState::TVector {
         /*** states (std::vector<CState> initializer list) ***/
         AddState<CStateSetLedColors>("set_deck_color", CBlockDemo::EColor::RED),
         AddState<CStateSetLiftActuatorPosition>("lower_lift_actuator", LIFT_ACTUATOR_MIN_HEIGHT + LIFT_ACTUATOR_BLOCK_PREATTACH_OFFSET),
         AddState<CState>("set_approach_velocity", [this] {
            double fLastObservationX = GetBase().GetData<SGlobalData>().TrackedTargetLastObservation.Translation.GetX();
            double fLeft = BASE_VELOCITY * (1.000 + (fLastObservationX * BASE_XZ_GAIN));
            double fRight = BASE_VELOCITY * (1.000 - (fLastObservationX * BASE_XZ_GAIN));
            GetBase().GetData<SGlobalData>().SetVelocity(fLeft, fRight);
         }),
         AddState<CState>("wait_for_underneath_rf_or_timeout"),
         AddState<CState>("wait_for_either_left_right_rf_or_timeout"),
         AddState<CState>("set_pivot_velocity", [this] {
            bool bRfBlockDetectedLeft = (GetBase().GetData<SGlobalData>().Sensors->ManipulatorModule.RangeFinders.Left > RF_LR_BLOCK_DETECT_THRES);
            bool bRfBlockDetectedRight = (GetBase().GetData<SGlobalData>().Sensors->ManipulatorModule.RangeFinders.Right > RF_LR_BLOCK_DETECT_THRES);
            // pivot the robot towards the other sensor
            double fLeft = (bRfBlockDetectedLeft ? 0.250 : 0.500) * BASE_VELOCITY;
            double fRight = (bRfBlockDetectedRight ? 0.250 : 0.500) * BASE_VELOCITY;
            // apply the velocity
            GetBase().GetData<SGlobalData>().SetVelocity(fLeft, fRight);
         }),
         AddState<CState>("wait_for_both_left_right_rf_or_timeout"),
         AddState<CStateSetVelocity>("set_zero_velocity", 0.000, 0.000),
      }) {
         /*** transitions (constructor body) ***/
         AddTransition("set_deck_color", "lower_lift_actuator");
         AddTransition("lower_lift_actuator", "set_approach_velocity");
         AddTransition("set_approach_velocity", "wait_for_underneath_rf_or_timeout", [this] {
            // reset timer for "wait_for_underneath_rf_or_timeout"
            GetBase().GetData<SGlobalData>().NearApproachStartTime = GetBase().GetData<SGlobalData>().Sensors->Clock.Time;
            return true;
         });
         AddTransition("wait_for_underneath_rf_or_timeout", "wait_for_either_left_right_rf_or_timeout", [this] {
            // block detected on the underneath rf
            if(GetBase().GetData<SGlobalData>().Sensors->ManipulatorModule.RangeFinders.Underneath > RF_UN_BLOCK_DETECT_THRES) {
               // reset timer for "wait_for_either_left_right_rf_or_timeout"
               GetBase().GetData<SGlobalData>().NearApproachStartTime = GetBase().GetData<SGlobalData>().Sensors->Clock.Time;
               return true;
            }
            return false;
         });
         AddTransition("wait_for_either_left_right_rf_or_timeout", "set_zero_velocity", [this] {
            // block detected on both the left & right rf
            return ((GetBase().GetData<SGlobalData>().Sensors->ManipulatorModule.RangeFinders.Left > RF_LR_BLOCK_DETECT_THRES) &&
                    (GetBase().GetData<SGlobalData>().Sensors->ManipulatorModule.RangeFinders.Right > RF_LR_BLOCK_DETECT_THRES));
         });
         AddTransition("wait_for_either_left_right_rf_or_timeout", "set_pivot_velocity", [this] {
            // block detected on both the left & right rf
            return ((GetBase().GetData<SGlobalData>().Sensors->ManipulatorModule.RangeFinders.Left > RF_LR_BLOCK_DETECT_THRES) ||
                    (GetBase().GetData<SGlobalData>().Sensors->ManipulatorModule.RangeFinders.Right > RF_LR_BLOCK_DETECT_THRES));
         });
         AddTransition("set_pivot_velocity", "wait_for_both_left_right_rf_or_timeout", [this] {
            // reset timer for "wait_for_both_left_right_rf_or_timeout"
            GetBase().GetData<SGlobalData>().NearApproachStartTime = GetBase().GetData<SGlobalData>().Sensors->Clock.Time;
            return true;
         });
         AddTransition("wait_for_both_left_right_rf_or_timeout", "set_zero_velocity", [this] {
            // block detected on both the left & right rf
            return ((GetBase().GetData<SGlobalData>().Sensors->ManipulatorModule.RangeFinders.Left > RF_LR_BLOCK_DETECT_THRES) &&
                    (GetBase().GetData<SGlobalData>().Sensors->ManipulatorModule.RangeFinders.Right > RF_LR_BLOCK_DETECT_THRES));
         });
         AddTransition("wait_for_underneath_rf_or_timeout", "set_zero_velocity", [this] {
            if(GetBase().GetData<SGlobalData>().NearApproachStartTime + NEAR_APPROACH_TIMEOUT < GetBase().GetData<SGlobalData>().Sensors->Clock.Time) {
               GetBase().GetData<SGlobalData>().ClearTargetInRange();
               return true;
            }
            return false;
         });
         AddTransition("wait_for_either_left_right_rf_or_timeout", "set_zero_velocity", [this] {
            if(GetBase().GetData<SGlobalData>().NearApproachStartTime + NEAR_APPROACH_TIMEOUT < GetBase().GetData<SGlobalData>().Sensors->Clock.Time) {
               GetBase().GetData<SGlobalData>().ClearTargetInRange();
               return true;
            }
            return false;
         });
         AddTransition("wait_for_both_left_right_rf_or_timeout", "set_zero_velocity", [this] {
            if(GetBase().GetData<SGlobalData>().NearApproachStartTime + NEAR_APPROACH_TIMEOUT < GetBase().GetData<SGlobalData>().Sensors->Clock.Time) {
               GetBase().GetData<SGlobalData>().ClearTargetInRange();
               return true;
            }
            return false;
         });
         AddExitTransition("set_zero_velocity");
      }
};

class CStatePickUpBlock : public CState {
public:
   CStatePickUpBlock(const std::string& str_id, CState* pc_parent) :
      CState(str_id, pc_parent, nullptr, CState::TVector {
         /*** states (std::vector<CState> initializer list) ***/
         AddState<CStateSetLedColors>("set_deck_color_green", CBlockDemo::EColor::GREEN),
         AddState<CStateMoveToTargetXZ>("align_with_block", PREAPPROACH_BLOCK_X_TARGET, PREAPPROACH_BLOCK_Z_TARGET, false),
         AddState<CStateApproachTargetFar>("approach_block_from_left", TAG_OFFSET_TARGET, FindTagCornerFurthestToTheRight),
         AddState<CStateApproachTargetFar>("approach_block_from_right", -TAG_OFFSET_TARGET, FindTagCornerFurthestToTheLeft),
         AddState<CStateApproachTargetFar>("approach_block_straight", 0.000, GetTagCenter),
         AddState<CStateSetLedColors>("set_deck_color_red", CBlockDemo::EColor::RED),
         AddState<CStateApproachTargetNear>("approach_block_near"),
         AddState<CStateSetVelocity>("set_zero_velocity", 0.000, 0.000),
         AddState<CStateAttachBlock>("attach_block_to_end_effector"),
         AddState<CStateSetLiftActuatorPosition>("set_lift_actuator_test_height", LIFT_ACTUATOR_MIN_HEIGHT + LIFT_ACTUATOR_BLOCK_HEIGHT),
         // failure states
         AddState<CStateSetVelocity>("set_reverse_velocity", -0.250 * BASE_VELOCITY, -0.250 * BASE_VELOCITY),
         AddState<CStateSetLiftActuatorPosition>("raise_lift_actuator", LIFT_ACTUATOR_MAX_HEIGHT),
      }) {
         /*** transitions (constructor body) ***/
         // select approach direction
         AddTransition("set_deck_color_green", "align_with_block");
         AddTransition("align_with_block", "approach_block_from_left", [this] {
            auto itTarget = FindTrackedTarget(GetBase().GetData<SGlobalData>().TrackedTargetId, GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets);
            if(itTarget != std::end(GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets)) {
               const SBlock& s_block = itTarget->Observations.front();
               if((std::abs(s_block.Translation.GetX() - PREAPPROACH_BLOCK_X_TARGET) < PREAPPROACH_BLOCK_XZ_THRES) &&
                  (std::abs(s_block.Translation.GetZ() - PREAPPROACH_BLOCK_Z_TARGET) < PREAPPROACH_BLOCK_XZ_THRES)) {
                  argos::CRadians cEulerAngleZ, cEulerAngleY, cEulerAngleX;
                  s_block.Rotation.ToEulerAngles(cEulerAngleZ, cEulerAngleY, cEulerAngleX);
                  return (cEulerAngleZ.GetValue() >= (M_PI / 18.0));
               }
            }
            return false;
         });
         AddTransition("align_with_block", "approach_block_from_right", [this] {
            auto itTarget = FindTrackedTarget(GetBase().GetData<SGlobalData>().TrackedTargetId, GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets);
            if(itTarget != std::end(GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets)) {
               const SBlock& s_block = itTarget->Observations.front();
               if((std::abs(s_block.Translation.GetX() - PREAPPROACH_BLOCK_X_TARGET) < PREAPPROACH_BLOCK_XZ_THRES) &&
                  (std::abs(s_block.Translation.GetZ() - PREAPPROACH_BLOCK_Z_TARGET) < PREAPPROACH_BLOCK_XZ_THRES)) {
                  argos::CRadians cEulerAngleZ, cEulerAngleY, cEulerAngleX;
                  s_block.Rotation.ToEulerAngles(cEulerAngleZ, cEulerAngleY, cEulerAngleX);
                  return (cEulerAngleZ.GetValue() <= -(M_PI / 18.0));
               }
            }
            return false;
         });
         AddTransition("align_with_block", "approach_block_straight", [this] {
            auto itTarget = FindTrackedTarget(GetBase().GetData<SGlobalData>().TrackedTargetId, GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets);
            if(itTarget != std::end(GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets)) {
               const SBlock& s_block = itTarget->Observations.front();
               if((std::abs(s_block.Translation.GetX() - PREAPPROACH_BLOCK_X_TARGET) < PREAPPROACH_BLOCK_XZ_THRES) &&
                  (std::abs(s_block.Translation.GetZ() - PREAPPROACH_BLOCK_Z_TARGET) < PREAPPROACH_BLOCK_XZ_THRES)) {
                  argos::CRadians cEulerAngleZ, cEulerAngleY, cEulerAngleX;
                  s_block.Rotation.ToEulerAngles(cEulerAngleZ, cEulerAngleY, cEulerAngleX);
                  return ((cEulerAngleZ.GetValue() > -(M_PI / 18.0)) && (cEulerAngleZ.GetValue() < (M_PI / 18.0)));
               }
            }
            return false;
         });

         // check if target is in range and perform the near block approach
         AddTransition("approach_block_from_left", "set_deck_color_red", [this] {
            return GetBase().GetData<SGlobalData>().IsTargetInRange();
         });
         AddTransition("approach_block_from_right", "set_deck_color_red", [this] {
            return GetBase().GetData<SGlobalData>().IsTargetInRange();
         });
         AddTransition("approach_block_straight", "set_deck_color_red", [this] {
            return GetBase().GetData<SGlobalData>().IsTargetInRange();
         });
         AddTransition("set_deck_color_red", "approach_block_near");
         AddTransition("approach_block_near", "set_zero_velocity", [this] {
            return GetBase().GetData<SGlobalData>().IsTargetInRange();
         });
         AddTransition("set_zero_velocity", "attach_block_to_end_effector");
         // test if the block attached correctly to the end effector
         AddTransition("attach_block_to_end_effector", "set_lift_actuator_test_height");
         AddExitTransition("set_lift_actuator_test_height", [this] {
            if(GetBase().GetData<SGlobalData>().Sensors->ManipulatorModule.RangeFinders.Underneath > RF_UN_BLOCK_DETECT_THRES) {
               GetBase().GetData<SGlobalData>().SetTargetInRange();
               return true;
            }
            return false;
         });
         // failure transitions
         AddTransition("align_with_block", "set_reverse_velocity", [this] {
            return GetBase().GetData<SGlobalData>().IsTargetLost();
         });
         AddTransition("approach_block_from_left", "set_reverse_velocity");
         AddTransition("approach_block_from_right", "set_reverse_velocity");
         AddTransition("approach_block_straight", "set_reverse_velocity");
         AddTransition("approach_block_near", "set_reverse_velocity");
         AddTransition("set_lift_actuator_test_height", "set_reverse_velocity");
         AddTransition("set_reverse_velocity", "raise_lift_actuator");
         AddExitTransition("raise_lift_actuator", [this] {
            return GetBase().GetData<SGlobalData>().ClearTargetInRange();
         });
      }
};

class CStateApproachStructure : public CState {
public:
   CStateApproachStructure(const std::string& str_id, CState* pc_parent, double f_tag_offset_target,
                           std::function<const STag::TCoordinate&(const STag&)> fn_get_coordinate) :
      CState(str_id, pc_parent, nullptr, CState::TVector {
         /*** states (std::vector<CState> initializer list) ***/
         AddState<CStateAlignWithTagOffset>("align_with_tag_offset", f_tag_offset_target, fn_get_coordinate),
         AddState<CStateApproachTarget>("approach_target", LIFT_ACTUATOR_MIN_HEIGHT + (0.5 * LIFT_ACTUATOR_BLOCK_HEIGHT), f_tag_offset_target, fn_get_coordinate),

         // failure states
         AddState<CState>("adjust_lift_actuator_height", [this] {
            if(GetBase().GetData<SGlobalData>().Actuators->ManipulatorModule.LiftActuator.Position.Value < (LIFT_ACTUATOR_MAX_HEIGHT - LIFT_ACTUATOR_BLOCK_PREATTACH_OFFSET)) {
               GetBase().GetData<SGlobalData>().Actuators->ManipulatorModule.LiftActuator.Position.Value += LIFT_ACTUATOR_BLOCK_PREATTACH_OFFSET;
               GetBase().GetData<SGlobalData>().Actuators->ManipulatorModule.LiftActuator.Position.UpdateReq = true;
            }
         }),
         AddState<CStateSetVelocity>("set_reverse_velocity", -0.250 * BASE_VELOCITY, -0.250 * BASE_VELOCITY),
         AddState<CState>("wait_for_target_or_timeout"),
      }) {
      /*** transitions (constructor body) ***/
      AddTransition("align_with_tag_offset", "approach_target", [this, f_tag_offset_target, fn_get_coordinate] {
         auto itTarget = FindTrackedTarget(GetBase().GetData<SGlobalData>().TrackedTargetId, GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets);
         if(itTarget != std::end(GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            double fTagOffset = (fn_get_coordinate(s_block.Tags[0]).first - IMAGE_SENSOR_HALF_WIDTH) / IMAGE_SENSOR_HALF_WIDTH;
            if(std::abs(fTagOffset - f_tag_offset_target) < 0.1) {
               GetBase().GetData<SGlobalData>().TagApproachController.Reset();
               return true;
            }
         }
         return false;
      });

      AddExitTransition("approach_target", [this] {
         auto itTarget = FindTrackedTarget(GetBase().GetData<SGlobalData>().TrackedTargetId, GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets);
         bool bTargetInRange = (itTarget != std::end(GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets)) && 
                               (GetAdjBlockTranslation(itTarget->Observations.front()).GetX() < 0.085);
         bool bTargetLost = GetBase().GetData<SGlobalData>().IsTargetLost();
         bool bLiftActuatorAtBottom =
            (GetBase().GetData<SGlobalData>().Actuators->ManipulatorModule.LiftActuator.Position.Value <= (LIFT_ACTUATOR_MIN_HEIGHT + (0.5 * LIFT_ACTUATOR_BLOCK_HEIGHT)));

         if(bTargetInRange || (bTargetLost && bLiftActuatorAtBottom)) {
            GetBase().GetData<SGlobalData>().SetTargetInRange();
            return true;
         }
         return false;
      });

      AddTransition("align_with_tag_offset", "set_reverse_velocity", [this] {
         return GetBase().GetData<SGlobalData>().IsTargetLost();
      });
      AddTransition("approach_target", "set_reverse_velocity", [this] {
         return GetBase().GetData<SGlobalData>().IsTargetLost();
      });
      AddTransition("set_reverse_velocity", "adjust_lift_actuator_height");
      // back off until target is re-acquired
      AddTransition("adjust_lift_actuator_height", "wait_for_target_or_timeout", [this] {
         // reset timer for the reverse velocity search
         GetBase().GetData<SGlobalData>().ReverseToFindTargetStartTime = GetBase().GetData<SGlobalData>().Sensors->Clock.Time;
         return true;
      });
      // try again
      AddTransition("wait_for_target_or_timeout", "align_with_tag_offset", [this] {
         return GetBase().GetData<SGlobalData>().IsNextTargetAcquired();
      });
      // timer has expired
      AddExitTransition("wait_for_target_or_timeout", [this] {
         if(GetBase().GetData<SGlobalData>().ReverseToFindTargetStartTime + REVERSE_TIMEOUT_SHORT < GetBase().GetData<SGlobalData>().Sensors->Clock.Time) {
            GetBase().GetData<SGlobalData>().ClearTargetInRange();
            return true;
         }
         return false;
      });
   }
};

class CStatePlaceBlock : public CState {
public:
   /* local data */
   std::chrono::time_point<std::chrono::steady_clock> m_tpIntervalStartTime;
   std::chrono::milliseconds m_tIntervalLength;
   argos::CRange<argos::UInt32> m_cIntervalLengthRange = argos::CRange<argos::UInt32>(0,30000);

   CStatePlaceBlock(const std::string& str_id, CState* pc_parent) :
      CState(str_id, pc_parent, nullptr, CState::TVector {
         /*** states (std::vector<CState> initializer list) ***/
         AddState<CStateSetLedColors>("set_deck_color_green", CBlockDemo::EColor::GREEN),
         AddState<CStateMoveToTargetXZ>("reobserve_structure", OBSERVE_BLOCK_X_TARGET, OBSERVE_BLOCK_Z_TARGET, true),
         AddState<CStateMoveToTargetXZ>("prealign_with_structure", PREAPPROACH_BLOCK_X_TARGET, PREAPPROACH_BLOCK_Z_TARGET, true),
         AddState<CStateSetVelocity>("set_standby_velocity", 0.000, 0.000),
         AddState<CState>("wait_for_approach_timer"),
         AddState<CStateApproachStructure>("approach_structure_from_left", TAG_OFFSET_TARGET, FindTagCornerFurthestToTheRight),
         AddState<CStateApproachStructure>("approach_structure_from_right", -TAG_OFFSET_TARGET, FindTagCornerFurthestToTheLeft),
         AddState<CStateApproachStructure>("approach_structure_straight", 0.000, GetTagCenter),
         AddState<CStateSetVelocity>("set_reverse_velocity", -0.250 * BASE_VELOCITY, -0.250 * BASE_VELOCITY),
         AddState<CState>("wait_for_target"),
         AddState<CStateMoveToTargetX>("align_with_structure", PREPLACEMENT_BLOCK_X_TARGET, false),
         AddState<CStateSetVelocity>("set_zero_velocity", 0.000, 0.000),
         AddState<CState>("wait_until_target_either_lr_clear"),
         AddState<CState>("wait_until_target_both_lr_clear"),
         AddState<CStateSetLedColors>("set_deck_color_red", CBlockDemo::EColor::RED),
         AddState<CStateSetLiftActuatorPosition>("set_lift_actuator_base_height", LIFT_ACTUATOR_MIN_HEIGHT + (0.5 * LIFT_ACTUATOR_BLOCK_HEIGHT)),
         // if no targets place block, otherwise, if targets check led colors
         AddState<CState>("increment_lift_actuator_height", [this] {
            GetBase().GetData<SGlobalData>().Actuators->ManipulatorModule.LiftActuator.Position.Value += LIFT_ACTUATOR_BLOCK_HEIGHT;
            /* saturate a max height */
            if(GetBase().GetData<SGlobalData>().Actuators->ManipulatorModule.LiftActuator.Position.Value > LIFT_ACTUATOR_MAX_HEIGHT) {
               GetBase().GetData<SGlobalData>().Actuators->ManipulatorModule.LiftActuator.Position.Value = LIFT_ACTUATOR_MAX_HEIGHT;
            }
            GetBase().GetData<SGlobalData>().Actuators->ManipulatorModule.LiftActuator.Position.UpdateReq = true;

         }),
         AddState<CState>("wait_for_lift_actuator"),
         AddState<CState>("decrement_lift_actuator_height", [this] {
            GetBase().GetData<SGlobalData>().Actuators->ManipulatorModule.LiftActuator.Position.Value -= (0.25 * LIFT_ACTUATOR_BLOCK_HEIGHT);
            GetBase().GetData<SGlobalData>().Actuators->ManipulatorModule.LiftActuator.Position.UpdateReq = true;
         }),
         AddState<CStateSetLiftActuatorPosition>("lower_lift_actuator", LIFT_ACTUATOR_MIN_HEIGHT + (0.25 * LIFT_ACTUATOR_BLOCK_HEIGHT)),
         AddState<CState>("set_approach_velocity", [this] {
            double fLastObservationX = GetBase().GetData<SGlobalData>().TrackedTargetLastObservation.Translation.GetX();
            double fLeft = BASE_VELOCITY * (1.000 + (fLastObservationX * BASE_XZ_GAIN));
            double fRight = BASE_VELOCITY * (1.000 - (fLastObservationX * BASE_XZ_GAIN));
            GetBase().GetData<SGlobalData>().SetVelocity(fLeft, fRight);
         }),
         AddState<CState>("wait_for_either_front_rf_or_timeout"),
         AddState<CState>("set_pivot_velocity", [this] {
            bool bRfBlockDetectedLeft = (GetMedian(GetBase().GetData<SGlobalData>().Sensors->RangeFinders[5]) > RF_FLR_BLOCK_DETECT_THRES);
            bool bRfBlockDetectedRight = (GetMedian(GetBase().GetData<SGlobalData>().Sensors->RangeFinders[6]) > RF_FLR_BLOCK_DETECT_THRES);
            // pivot the robot towards the other sensor
            double fLeft = (bRfBlockDetectedLeft ? 0.250 : 0.500) * BASE_VELOCITY;
            double fRight = (bRfBlockDetectedRight ? 0.250 : 0.500) * BASE_VELOCITY;
            // apply the velocity
            GetBase().GetData<SGlobalData>().SetVelocity(fLeft, fRight);
         }),
         AddState<CState>("wait_for_both_front_rfs_or_timeout"),
         AddState<CStateSetVelocity>("set_reverse_velocity_for_detachment", -0.500 * BASE_VELOCITY, -0.500 * BASE_VELOCITY),
         AddState<CState>("set_block_led_state", [this] {
            switch(GetBase().GetData<SGlobalData>().NextLedStateToAssign) {
            case ELedState::OFF:
               GetBase().GetData<SGlobalData>().Actuators->ManipulatorModule.NFCInterface.OutboundMessage = BLOCK_TYPE_OFF;
               break;
            case ELedState::Q1:
               GetBase().GetData<SGlobalData>().Actuators->ManipulatorModule.NFCInterface.OutboundMessage = BLOCK_TYPE_Q1;
               break;
            case ELedState::Q2:
               GetBase().GetData<SGlobalData>().Actuators->ManipulatorModule.NFCInterface.OutboundMessage = BLOCK_TYPE_Q2;
               break;
            case ELedState::Q3:
               GetBase().GetData<SGlobalData>().Actuators->ManipulatorModule.NFCInterface.OutboundMessage = BLOCK_TYPE_Q3;
               break;
            case ELedState::Q4:
               GetBase().GetData<SGlobalData>().Actuators->ManipulatorModule.NFCInterface.OutboundMessage = BLOCK_TYPE_Q4;
               break;
            }
            GetBase().GetData<SGlobalData>().Actuators->ManipulatorModule.NFCInterface.UpdateReq = true;
         }),
         AddState<CStatePulseElectromagnets>("deattach_block_from_end_effector", std::chrono::milliseconds(1000), CBlockDemo::EGripperFieldMode::DESTRUCTIVE),
         // Failure / completion states

         
      }) {
         /*** transitions (constructor body) ***/
         // select approach direction
         AddTransition("set_deck_color_green", "prealign_with_structure");
         AddTransition("reobserve_structure", "prealign_with_structure", [this] {
            auto itTarget = FindTrackedTarget(GetBase().GetData<SGlobalData>().TrackedTargetId, GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets);
            auto itQualitativeTarget = FindQualitativeTarget(GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets);
            if(itQualitativeTarget != std::end(GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets)) {
               if(itQualitativeTarget != itTarget) {
                  std::cerr << "switching from target (" << itTarget->Id << ") to qualitative target (" << itQualitativeTarget->Id << ")" << std::endl;
                  GetBase().GetData<SGlobalData>().TrackedTargetId = itQualitativeTarget->Id;
               }
            }
            if(itTarget != std::end(GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets)) {
               const SBlock& s_block = itTarget->Observations.front();
               if((std::abs(s_block.Translation.GetX() - OBSERVE_BLOCK_X_TARGET) < OBSERVE_BLOCK_XZ_THRES) &&
                  (std::abs(s_block.Translation.GetZ() - OBSERVE_BLOCK_Z_TARGET) < OBSERVE_BLOCK_XZ_THRES)) {
                  return true;
               }
            }
            return false;
         });

         AddTransition("prealign_with_structure","set_standby_velocity", [this] {
            auto itTarget = FindTrackedTarget(GetBase().GetData<SGlobalData>().TrackedTargetId, GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets);
            auto itQualitativeTarget = FindQualitativeTarget(GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets);
            if(itQualitativeTarget != std::end(GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets)) {
               if(itQualitativeTarget != itTarget) {
                  std::cerr << "switching from target (" << itTarget->Id << ") to qualitative target (" << itQualitativeTarget->Id << ")" << std::endl;
                  GetBase().GetData<SGlobalData>().TrackedTargetId = itQualitativeTarget->Id;
               }
            }
            if(itTarget != std::end(GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets)) {
               const SBlock& s_block = itTarget->Observations.front();
               if((std::abs(s_block.Translation.GetX() - PREAPPROACH_BLOCK_X_TARGET) < PREAPPROACH_BLOCK_XZ_THRES) &&
                  (std::abs(s_block.Translation.GetZ() - PREAPPROACH_BLOCK_Z_TARGET) < PREAPPROACH_BLOCK_XZ_THRES)) {
                  return true;
               }
            }
            return false;
         });

         AddTransition("set_standby_velocity", "wait_for_approach_timer", [this] {
            m_tpIntervalStartTime = GetBase().GetData<SGlobalData>().Sensors->Clock.Time;
            argos::UInt32 unRandomInterval = 0;
            m_tIntervalLength = std::chrono::milliseconds(unRandomInterval);
            return true;
         });

         AddTransition("wait_for_approach_timer","reobserve_structure", [this] {
            return (FindTrackedTarget(GetBase().GetData<SGlobalData>().TrackedTargetId, GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets) !=
               FindQualitativeTarget(GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets));
         });

         AddTransition("wait_for_approach_timer","approach_structure_from_left", [this] {
            if(m_tpIntervalStartTime + m_tIntervalLength < GetBase().GetData<SGlobalData>().Sensors->Clock.Time) {
               auto itTarget = FindTrackedTarget(GetBase().GetData<SGlobalData>().TrackedTargetId, GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets);
               if(itTarget != std::end(GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets)) {
                  argos::CRadians cEulerAngleZ, cEulerAngleY, cEulerAngleX;
                  itTarget->Observations.front().Rotation.ToEulerAngles(cEulerAngleZ, cEulerAngleY, cEulerAngleX);
                  return (cEulerAngleZ.GetValue() >= (M_PI / 18.0));
               }
            }
            return false;
         });

         AddTransition("wait_for_approach_timer","approach_structure_from_right", [this] {
            if(m_tpIntervalStartTime + m_tIntervalLength < GetBase().GetData<SGlobalData>().Sensors->Clock.Time) {
               auto itTarget = FindTrackedTarget(GetBase().GetData<SGlobalData>().TrackedTargetId, GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets);
               if(itTarget != std::end(GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets)) {
                  argos::CRadians cEulerAngleZ, cEulerAngleY, cEulerAngleX;
                  itTarget->Observations.front().Rotation.ToEulerAngles(cEulerAngleZ, cEulerAngleY, cEulerAngleX);
                  return (cEulerAngleZ.GetValue() <= -(M_PI / 18.0));
               }
            }
            return false;
         });

         AddTransition("wait_for_approach_timer","approach_structure_straight", [this] {
            if(m_tpIntervalStartTime + m_tIntervalLength < GetBase().GetData<SGlobalData>().Sensors->Clock.Time) {
               auto itTarget = FindTrackedTarget(GetBase().GetData<SGlobalData>().TrackedTargetId, GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets);
               if(itTarget != std::end(GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets)) {
                  argos::CRadians cEulerAngleZ, cEulerAngleY, cEulerAngleX;
                  itTarget->Observations.front().Rotation.ToEulerAngles(cEulerAngleZ, cEulerAngleY, cEulerAngleX);
                  return ((cEulerAngleZ.GetValue() > -(M_PI / 18.0)) && (cEulerAngleZ.GetValue() < (M_PI / 18.0)));
               }
            }
            return false;
         });

         /* tracking lost - exit */
         AddExitTransition("prealign_with_structure", [this] {
            return GetBase().GetData<SGlobalData>().IsTargetLost();
         });
         AddExitTransition("wait_for_approach_timer", [this] {
            return GetBase().GetData<SGlobalData>().IsTargetLost();
         });
         AddTransition("approach_structure_from_left","set_reverse_velocity", [this] {
            return GetBase().GetData<SGlobalData>().IsTargetLost();
         });
         AddTransition("approach_structure_from_right","set_reverse_velocity", [this] {
            return GetBase().GetData<SGlobalData>().IsTargetLost();
         });
         AddTransition("approach_structure_straight","set_reverse_velocity", [this] {
            return GetBase().GetData<SGlobalData>().IsTargetLost();
         });
         AddTransition("approach_structure_from_left", "align_with_structure", [this] {
            auto itTarget = FindTrackedTarget(GetBase().GetData<SGlobalData>().TrackedTargetId, GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets);
            if(itTarget != std::end(GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets)) {
               const SBlock& s_block = itTarget->Observations.front();
               std::cerr << "approach_structure_from_left.GetAdjBlockTranslation(" << itTarget->Id << ").GetX() = " << GetAdjBlockTranslation(s_block).GetX() << std::endl;
            }
            else {
               std::cerr << "approach_structure_from_left: target not found" << std::endl;
            }
            return true;
         });
         AddTransition("approach_structure_from_right", "align_with_structure", [this] {
            auto itTarget = FindTrackedTarget(GetBase().GetData<SGlobalData>().TrackedTargetId, GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets);
            if(itTarget != std::end(GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets)) {
               const SBlock& s_block = itTarget->Observations.front();
               std::cerr << "approach_structure_from_right.GetAdjBlockTranslation(" << itTarget->Id << ").GetX() = " << GetAdjBlockTranslation(s_block).GetX() << std::endl;
            }
            else {
               std::cerr << "approach_structure_from_right: target not found" << std::endl;
            }
            return true;
         });
         AddTransition("approach_structure_straight", "align_with_structure", [this] {
            auto itTarget = FindTrackedTarget(GetBase().GetData<SGlobalData>().TrackedTargetId, GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets);
            if(itTarget != std::end(GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets)) {
               const SBlock& s_block = itTarget->Observations.front();
               std::cerr << "approach_structure_straight.GetAdjBlockTranslation(" << itTarget->Id << ").GetX() = " << GetAdjBlockTranslation(s_block).GetX() << std::endl;
            }
            else {
               std::cerr << "approach_structure_straight: target not found" << std::endl;
            }
            return true;
         });
         AddTransition("set_reverse_velocity", "wait_for_target");

         AddTransition("wait_for_target", "align_with_structure", [this] {
            return (GetBase().GetData<SGlobalData>().IsNextTargetAcquired() && GetBase().GetData<SGlobalData>().IsTargetInRange());
         });

         AddTransition("wait_for_target", "reobserve_structure", [this] {
            auto itTarget = FindTrackedTarget(GetBase().GetData<SGlobalData>().TrackedTargetId, GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets);
            if(itTarget != std::end(GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets)) {
               return true;
            }
            else if(GetBase().GetData<SGlobalData>().IsNextTargetAcquired()) {
               return true;
            }
            /*
            else if(GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets.size() > 0) {
               GetBase().GetData<SGlobalData>().TrackedTargetId = std::begin(GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets)->Id;
               return true;
            }
            */
            else {
               return false;
            }
         });
         AddTransition("align_with_structure", "set_zero_velocity", [this] {
            auto itTarget = FindTrackedTarget(GetBase().GetData<SGlobalData>().TrackedTargetId, GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets);
            if(itTarget != std::end(GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets)) {
               const SBlock& s_block = itTarget->Observations.front();
               if(std::abs(s_block.Translation.GetX() - PREPLACEMENT_BLOCK_X_TARGET) < PREPLACEMENT_BLOCK_X_THRES) {
                  argos::CRadians cEulerAngleZ, cEulerAngleY, cEulerAngleX;
                  s_block.Rotation.ToEulerAngles(cEulerAngleZ, cEulerAngleY, cEulerAngleX);
                  return ((cEulerAngleZ.GetValue() > -(M_PI / 18.0)) && (cEulerAngleZ.GetValue() < (M_PI / 18.0)));
               }
            }
            return false;
         });
         // loop back if alignment is to far out (more than +/- 10 deg)
         AddTransition("align_with_structure", "reobserve_structure", [this] {
            auto itTarget = FindTrackedTarget(GetBase().GetData<SGlobalData>().TrackedTargetId, GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets);
            if(itTarget != std::end(GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets)) {
               const SBlock& s_block = itTarget->Observations.front();
               if(std::abs(s_block.Translation.GetX() - PREPLACEMENT_BLOCK_X_TARGET) < PREPLACEMENT_BLOCK_X_THRES) {
                  return true;
               }
            }
            return false;
         });
         // retry if target is lost
         AddTransition("align_with_structure", "set_reverse_velocity", [this] {
            return GetBase().GetData<SGlobalData>().IsTargetLost();
         });

         AddTransition("set_zero_velocity", "set_reverse_velocity", [this] {
            /* other robots on the left and/or right detected */
            bool bLeftBlocked = (GetMedian(GetBase().GetData<SGlobalData>().Sensors->RangeFinders[4]) > RF_FCNR_ROBOT_DETECT_THRES);
            bool bRightBlocked = (GetMedian(GetBase().GetData<SGlobalData>().Sensors->RangeFinders[7]) > RF_FCNR_ROBOT_DETECT_THRES);
            if(bLeftBlocked || bRightBlocked) {
               GetBase().GetData<SGlobalData>().ClearTargetInRange();
               return true;
            }
            return false;
         });

         AddTransition("set_zero_velocity", "set_deck_color_red");
         AddTransition("set_deck_color_red", "set_lift_actuator_base_height");
         AddTransition("set_lift_actuator_base_height", "wait_for_lift_actuator");
         AddTransition("increment_lift_actuator_height", "wait_for_lift_actuator");

         // extend structure vertically
         AddTransition("wait_for_lift_actuator", "increment_lift_actuator_height", [this] {
            if(GetBase().GetData<SGlobalData>().Sensors->ManipulatorModule.LiftActuator.State == CBlockDemo::ELiftActuatorSystemState::INACTIVE) {
               for(const STarget& s_target : GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets) {
                  const SBlock& s_block = s_target.Observations.front();
                  if((GetAdjBlockTranslation(s_block).GetX() < 0.100) && (GetBlockLedState(s_block) == ELedState::Q1)) {
                     GetBase().GetData<SGlobalData>().NextLedStateToAssign = ELedState::OFF;
                     return true;
                  }
               }
            }
            return false;
         });

         // extend structure horizontally
         AddTransition("wait_for_lift_actuator", "lower_lift_actuator", [this] {
            if(GetBase().GetData<SGlobalData>().Sensors->ManipulatorModule.LiftActuator.State == CBlockDemo::ELiftActuatorSystemState::INACTIVE) {
               for(const STarget& s_target : GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets) {
                  const SBlock& s_block = s_target.Observations.front();
                  if(GetAdjBlockTranslation(s_block).GetX() < 0.100) {
                     ELedState eBlockLedState = GetBlockLedState(s_block);
                     unsigned int unBlockLevel = GetBlockLevel(s_block, GetBase().GetData<SGlobalData>().Sensors->ManipulatorModule.LiftActuator.EndEffector.Position);
                     if((eBlockLedState == ELedState::Q3) && (unBlockLevel == 2u)) {
                        GetBase().GetData<SGlobalData>().NextLedStateToAssign = ELedState::Q2;
                        return true;
                     }
                     if((eBlockLedState == ELedState::Q2) && (unBlockLevel == 1u)) {
                        GetBase().GetData<SGlobalData>().NextLedStateToAssign = ELedState::Q1;
                        return true;
                     }
                  }
               }
            }
            return false;
         });

         AddTransition("wait_for_lift_actuator", "decrement_lift_actuator_height", [this] {
            if(GetBase().GetData<SGlobalData>().Sensors->ManipulatorModule.LiftActuator.State == CBlockDemo::ELiftActuatorSystemState::INACTIVE) {
               for(const STarget& s_target : GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets) {
                  const SBlock& s_block = s_target.Observations.front();
                  if(GetAdjBlockTranslation(s_block).GetX() < 0.100) {
                     return false;
                  }
               }
               return true;
            }
            return false;
         });
         AddTransition("decrement_lift_actuator_height", "set_approach_velocity", [this] {
            return (GetBase().GetData<SGlobalData>().Sensors->ManipulatorModule.LiftActuator.State == CBlockDemo::ELiftActuatorSystemState::INACTIVE);
         });
         AddTransition("lower_lift_actuator", "set_approach_velocity", [this] {
            return (GetBase().GetData<SGlobalData>().Sensors->ManipulatorModule.LiftActuator.State == CBlockDemo::ELiftActuatorSystemState::INACTIVE);
         });
         AddTransition("set_approach_velocity", "wait_for_either_front_rf_or_timeout", [this] {
            // reset timer for "wait_for_either_front_rf_or_timeout"
            GetBase().GetData<SGlobalData>().NearApproachStartTime = GetBase().GetData<SGlobalData>().Sensors->Clock.Time;
            return true;
         });
         AddTransition("wait_for_either_front_rf_or_timeout", "set_reverse_velocity_for_detachment", [this] {
            if(GetBase().GetData<SGlobalData>().Actuators->ManipulatorModule.LiftActuator.Position.Value > LIFT_ACTUATOR_BLOCK_HEIGHT) {
               /* stack */
               return ((GetMedian(GetBase().GetData<SGlobalData>().Sensors->RangeFinders[5]) > RF_FLR_BLOCK_CONTACT_THRES) ||
                       (GetMedian(GetBase().GetData<SGlobalData>().Sensors->RangeFinders[6]) > RF_FLR_BLOCK_CONTACT_THRES));
            }
            else {
               /* extend */
               return (GetBase().GetData<SGlobalData>().Sensors->ManipulatorModule.RangeFinders.Front > RF_LR_BLOCK_DETECT_THRES);
            }
         });
         AddTransition("wait_for_either_front_rf_or_timeout", "set_pivot_velocity", [this] {
            if(GetBase().GetData<SGlobalData>().Actuators->ManipulatorModule.LiftActuator.Position.Value > LIFT_ACTUATOR_BLOCK_HEIGHT) {
               /* stack */
               return ((GetMedian(GetBase().GetData<SGlobalData>().Sensors->RangeFinders[5]) > RF_FLR_BLOCK_DETECT_THRES) ||
                       (GetMedian(GetBase().GetData<SGlobalData>().Sensors->RangeFinders[6]) > RF_FLR_BLOCK_DETECT_THRES));
            }
            return false;
         });
         AddTransition("set_pivot_velocity", "wait_for_both_front_rfs_or_timeout", [this] {
            // reset timer for "wait_for_both_front_rfs_or_timeout"
            GetBase().GetData<SGlobalData>().NearApproachStartTime = GetBase().GetData<SGlobalData>().Sensors->Clock.Time;
            return true;
         });
         AddTransition("wait_for_both_front_rfs_or_timeout", "set_reverse_velocity_for_detachment", [this] {
            return ((GetMedian(GetBase().GetData<SGlobalData>().Sensors->RangeFinders[5]) > RF_FLR_BLOCK_CONTACT_THRES) ||
                    (GetMedian(GetBase().GetData<SGlobalData>().Sensors->RangeFinders[6]) > RF_FLR_BLOCK_CONTACT_THRES));
         });

         AddTransition("wait_for_either_front_rf_or_timeout", "set_reverse_velocity", [this] {
            for(const STarget& s_target : GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets) {
               const SBlock& s_block = s_target.Observations.front();
               if(GetBlockLedState(s_block) == ELedState::Q4) {
                  GetBase().GetData<SGlobalData>().ClearTargetInRange();
                  return true;
               }
            }
            return false;
         });

         AddTransition("wait_for_both_front_rfs_or_timeout", "set_reverse_velocity", [this] {
            for(const STarget& s_target : GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets) {
               const SBlock& s_block = s_target.Observations.front();
               if(GetBlockLedState(s_block) == ELedState::Q4) {
                  GetBase().GetData<SGlobalData>().ClearTargetInRange();
                  return true;
               }
            }
            return false;           
         });

         AddTransition("set_reverse_velocity_for_detachment", "set_block_led_state");
         AddTransition("set_block_led_state", "deattach_block_from_end_effector");
         AddExitTransition("deattach_block_from_end_effector");

         // Error transitions
         AddExitTransition("wait_for_either_front_rf_or_timeout", [this] {
            if(GetBase().GetData<SGlobalData>().NearApproachStartTime + NEAR_APPROACH_TIMEOUT < GetBase().GetData<SGlobalData>().Sensors->Clock.Time) {
               return true;
            }
            return false;
         });
         AddExitTransition("wait_for_both_front_rfs_or_timeout", [this] {
            if(GetBase().GetData<SGlobalData>().NearApproachStartTime + NEAR_APPROACH_TIMEOUT < GetBase().GetData<SGlobalData>().Sensors->Clock.Time) {
               return true;
            }
            return false;
         });

      }
};

/************************************************************/
/*             Main state machine definition                */
/************************************************************/
class CFiniteStateMachine : public CState {
public:
   CFiniteStateMachine(CBlockDemo::SSensorData* ps_sensor_data, CBlockDemo::SActuatorData* ps_actuator_data) :
      CState("top_level_state", nullptr, nullptr, CState::TVector {
         AddState<CState>("search_for_unused_block", nullptr, CState::TVector {
            AddState<CStateSetLedColors>("set_deck_color", CBlockDemo::EColor::BLUE),
            AddState<CStateSetLiftActuatorPosition>("raise_lift_actuator", LIFT_ACTUATOR_MAX_HEIGHT),
            AddState<CStateSetVelocity>("set_search_velocity", BASE_VELOCITY * 0.500, -BASE_VELOCITY * 0.500),
            AddState<CState>("wait_for_next_target"),
            AddState<CStateMoveToTargetXZ>("align_with_block", PREAPPROACH_BLOCK_X_TARGET, PREAPPROACH_BLOCK_Z_TARGET, false),
         }),
         AddState<CStatePickUpBlock>("pick_up_unused_block"),
         // Assign the transport color to the block
         AddState<CStateSendNFCMessage>("configure_block_for_transport", BLOCK_TYPE_Q4),
         AddState<CState>("search_for_structure", nullptr, CState::TVector {
            AddState<CStateSetLedColors>("set_deck_color", CBlockDemo::EColor::BLUE),
            AddState<CStateSetLiftActuatorPosition>("raise_lift_actuator", LIFT_ACTUATOR_MAX_HEIGHT),
            AddState<CStateSetVelocity>("set_search_velocity", BASE_VELOCITY * 0.500, -BASE_VELOCITY * 0.500),
            AddState<CState>("wait_for_next_target"),
            AddState<CStateMoveToTargetXZ>("align_with_block", OBSERVE_BLOCK_X_TARGET, OBSERVE_BLOCK_Z_TARGET, false), // initial check if this is a seed block / structure
            // select closest ground level target to robot
         }),
         AddState<CStatePlaceBlock>("place_block_into_structure"),
         AddState<CStateSetVelocity>("set_reverse_velocity", -BASE_VELOCITY * 0.250, -BASE_VELOCITY * 0.250),
         AddState<CStateSetLiftActuatorPosition>("raise_lift_actuator", LIFT_ACTUATOR_MAX_HEIGHT),
         AddState<CState>("wait_for_next_target"),
         AddState<CStateSetVelocity>("set_search_velocity", BASE_VELOCITY * 0.500, -BASE_VELOCITY * 0.500),
         AddState<CState>("reduce_velocity", [this] {
            GetBase().GetData<SGlobalData>().Actuators->DifferentialDriveSystem.Left.Velocity = 
               std::round(GetBase().GetData<SGlobalData>().Actuators->DifferentialDriveSystem.Left.Velocity * 0.495);
            GetBase().GetData<SGlobalData>().Actuators->DifferentialDriveSystem.Right.Velocity = 
               std::round(GetBase().GetData<SGlobalData>().Actuators->DifferentialDriveSystem.Right.Velocity * 0.495);
            GetBase().GetData<SGlobalData>().Actuators->DifferentialDriveSystem.Left.UpdateReq = true;
            GetBase().GetData<SGlobalData>().Actuators->DifferentialDriveSystem.Right.UpdateReq = true;
         }),
      }) {

      /* initialize the local data */
      SetData<SGlobalData>(ps_sensor_data, ps_actuator_data);

      /**************** search_for_unused_block transitions ****************/
      GetState("search_for_unused_block").AddTransition("set_deck_color", "raise_lift_actuator");
      GetState("search_for_unused_block").AddTransition("raise_lift_actuator", "set_search_velocity");
      GetState("search_for_unused_block").AddTransition("set_search_velocity","wait_for_next_target");
      GetState("search_for_unused_block").AddTransition("wait_for_next_target", "align_with_block", [this] {
         return GetBase().GetData<SGlobalData>().IsNextTargetAcquired();
      });
      /* keep searching if the target was lost */
      GetState("search_for_unused_block").AddTransition("align_with_block", "set_search_velocity", [this] {
         return GetBase().GetData<SGlobalData>().IsTargetLost();
      });
      /* unused block found - exit */
      GetState("search_for_unused_block").AddExitTransition("align_with_block", [this] {
         auto itTarget = FindTrackedTarget(GetBase().GetData<SGlobalData>().TrackedTargetId, GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets);
         if(itTarget != std::end(GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            if((std::abs(s_block.Translation.GetX() - PREAPPROACH_BLOCK_X_TARGET) < PREAPPROACH_BLOCK_XZ_THRES) &&
               (std::abs(s_block.Translation.GetZ() - PREAPPROACH_BLOCK_Z_TARGET) < PREAPPROACH_BLOCK_XZ_THRES)) {
               /* search all structures, does it belong to any */
               for(const SStructure& s_structure : GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Structures) {
                  if(std::find(std::begin(s_structure.Members), std::end(s_structure.Members), itTarget) != std::end(s_structure.Members)) {
                     /* is the size of the structure 1? i.e. a unused block */
                     return (s_structure.Members.size() == 1);
                  }
               }
            }
         }
         return false;
      });
      // keep searching if block belongs to a structure */
      GetState("search_for_unused_block").AddTransition("align_with_block", "set_search_velocity", [this] {
         auto itTarget = FindTrackedTarget(GetBase().GetData<SGlobalData>().TrackedTargetId, GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets);
         if(itTarget != std::end(GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            if((std::abs(s_block.Translation.GetX() - PREAPPROACH_BLOCK_X_TARGET) < PREAPPROACH_BLOCK_XZ_THRES) &&
               (std::abs(s_block.Translation.GetZ() - PREAPPROACH_BLOCK_Z_TARGET) < PREAPPROACH_BLOCK_XZ_THRES)) {
               // search all structures, does it belong to any */
               for(const SStructure& s_structure : GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Structures) {
                  if(std::find(std::begin(s_structure.Members), std::end(s_structure.Members), itTarget) != std::end(s_structure.Members)) {
                     // is the size of the structure not 1?
                     return (s_structure.Members.size() != 1);
                  }
               }
            }
         }
         return false;
      });

      /// Top level transitions ///
      AddTransition("search_for_unused_block", "pick_up_unused_block");
      AddTransition("pick_up_unused_block", "configure_block_for_transport", [this] {
         return (GetBase().GetData<SGlobalData>().Sensors->ManipulatorModule.RangeFinders.Underneath > RF_UN_BLOCK_DETECT_THRES);
      });
      AddTransition("configure_block_for_transport", "search_for_structure");

      /**************** search_for_structure transitions ****************/
      GetState("search_for_structure").AddTransition("set_deck_color", "raise_lift_actuator");
      GetState("search_for_structure").AddTransition("raise_lift_actuator", "set_search_velocity");
      GetState("search_for_structure").AddTransition("set_search_velocity", "wait_for_next_target");
      GetState("search_for_structure").AddTransition("wait_for_next_target", "align_with_block", [this] {
         return GetBase().GetData<SGlobalData>().IsNextTargetAcquired();
      });
      GetState("search_for_structure").AddExitTransition("align_with_block", [this] {
         auto itTarget = FindTrackedTarget(GetBase().GetData<SGlobalData>().TrackedTargetId, GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets);
         if(itTarget != std::end(GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            if((std::abs(s_block.Translation.GetX() - OBSERVE_BLOCK_X_TARGET) < OBSERVE_BLOCK_XZ_THRES) &&
               (std::abs(s_block.Translation.GetZ() - OBSERVE_BLOCK_Z_TARGET) < OBSERVE_BLOCK_XZ_THRES)) {
               // search all structures, does it belong to any
               for(const SStructure& s_structure : GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Structures) {
                  if(std::find(std::begin(s_structure.Members), std::end(s_structure.Members), itTarget) != std::end(s_structure.Members)) {
                     if(s_structure.Members.size() == 1) {
                        /* this target is a seed block */
                        return (GetLedCount(s_block, {ELedState::OFF}) < 4);
                     }
                     else {
                        /* this target is the correct target in a partially completed structure */
                        return (itTarget == FindQualitativeTarget(GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets));
                     }
                  }
               }
            }
         }
         return false;
      });
      // keep searching if block does not belong to a structure / is the seed block
      GetState("search_for_structure").AddTransition("align_with_block", "set_search_velocity", [this] {
         auto itTarget = FindTrackedTarget(GetBase().GetData<SGlobalData>().TrackedTargetId, GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets);
         if(itTarget != std::end(GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets)) {
            const SBlock& s_block = itTarget->Observations.front();
            if((std::abs(s_block.Translation.GetX() - OBSERVE_BLOCK_X_TARGET) < OBSERVE_BLOCK_XZ_THRES) &&
               (std::abs(s_block.Translation.GetZ() - OBSERVE_BLOCK_Z_TARGET) < OBSERVE_BLOCK_XZ_THRES)) {
               // if the above transition did not occur, then this is not our target
               auto itQualitativeTarget = FindQualitativeTarget(GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets);
               if(itQualitativeTarget != std::end(GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets) && itQualitativeTarget != itTarget) {
                  GetBase().GetData<SGlobalData>().TrackedTargetId = itQualitativeTarget->Id;
                  return false;
               }
               else {
                  return !GetBase().GetData<SGlobalData>().IsNextTargetAcquired();
               }
            }
         }
         return false;
      });
      // keep searching if target is lost
      GetState("search_for_structure").AddTransition("align_with_block", "set_search_velocity", [this] {
            return GetBase().GetData<SGlobalData>().IsTargetLost();
      });
      /// Top level transitions ///
      AddTransition("search_for_structure", "place_block_into_structure");
      // Error handling
      AddTransition("pick_up_unused_block", "set_reverse_velocity");
      AddTransition("place_block_into_structure", "set_reverse_velocity");
      AddTransition("set_reverse_velocity", "wait_for_next_target");
      // Reverse the right amount by locating a block placed in the most recent column
      AddTransition("wait_for_next_target", "reduce_velocity", [this] {
         for(const STarget& s_target : GetBase().GetData<SGlobalData>().Sensors->ImageSensor.Detections.Targets) {
            const SBlock& s_block = s_target.Observations.front();
            return (GetBlockLedState(s_block) == ELedState::Q1);
         }
         return false;
      });
   }
};

#endif
