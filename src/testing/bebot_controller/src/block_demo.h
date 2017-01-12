#ifndef BLOCK_DEMO_H
#define BLOCK_DEMO_H

#include <cstdint>
#include <array>
#include <list>
#include <chrono>

#include "block.h"
#include "target.h"
#include "structure.h"

#include <argos3/core/utility/math/rng.h>

/* compatibility hack */
class CBlockDemo {
public:
   enum class EGripperFieldMode : uint8_t {
      CONSTRUCTIVE = 0,
      DESTRUCTIVE = 1,
      DISABLED = 2
   };

   enum class ELiftActuatorSystemState : uint8_t {
      /* Inactive means the stepper motor is disabled */
      INACTIVE = 0,
      /* Active means the stepper motor is running */
      ACTIVE_POSITION_CTRL = 1,
      ACTIVE_SPEED_CTRL = 2,
      /* Calibration search bottom/top */
      CALIBRATION_SRCH_TOP = 3,
      CALIBRATION_SRCH_BTM = 4,
      /* Not actually a state */
      UNDEFINED = 5,
   };

   enum class EColor {
      RED, GREEN, BLUE
   };

   struct SSensorData {
      struct {
         struct {
            STarget::TList Targets;
            SStructure::TList Structures;
         } Detections;
      } ImageSensor;
      struct {
         struct {
            uint16_t EndEffector = 0, Left = 0, Right = 0,
               Front = 0, Underneath = 0;
         } RangeFinders;
         struct {
            struct {
               bool Top = false, Bottom = false;
            } LimitSwitches;
            struct {
               std::list<uint8_t> Charge;
            } Electromagnets;
            struct {
               uint8_t Position = 0;
            } EndEffector;
            ELiftActuatorSystemState State = ELiftActuatorSystemState::UNDEFINED;
         } LiftActuator;
      } ManipulatorModule;
      struct {
         std::chrono::time_point<std::chrono::steady_clock> ExperimentStart;
         std::chrono::time_point<std::chrono::steady_clock> Time;
         unsigned int Ticks = 0;
      } Clock;
      std::array<std::list<uint16_t>, 12> RangeFinders;
   };

   struct SActuatorData {
      struct {
         struct {
            int16_t Velocity = 0;
            bool UpdateReq = false;
         } Left, Right;
      } DifferentialDriveSystem;
      struct {
         std::array<EColor, 12> Color;
         std::array<bool, 12> UpdateReq = {};
      } LEDDeck;
      struct {
         struct {
            struct {
               int8_t Value = 0;
               bool UpdateReq = false;
            } Velocity;
            struct {
               uint8_t Value = 0;
               bool UpdateReq = false;
            } Position;
         } LiftActuator;
         struct {
            std::string OutboundMessage;
            bool UpdateReq = false;
         } NFCInterface;
         struct {
            EGripperFieldMode FieldMode =
               EGripperFieldMode::DISABLED;
            bool UpdateReq = false;
         } EndEffector;
      } ManipulatorModule;
   };
};

#endif
