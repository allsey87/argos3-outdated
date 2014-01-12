/**
 * @file <argos3/plugins/robots/prototype/simulator/prototype_proximity_sensor_equipped_entity.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef PROTOTYPE_PROXIMITY_SENSOR_EQUIPPED_ENTITY_H
#define PROTOTYPE_PROXIMITY_SENSOR_EQUIPPED_ENTITY_H

namespace argos {
   class CPrototypeProximitySensorEquippedEntity;
}

#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/simulator/entity/entity.h>
#include <argos3/plugins/robots/prototype/simulator/body_entity.h>

#include <vector>

namespace argos {

   class CPrototypeProximitySensorEquippedEntity : public CEntity {

   public:

      ENABLE_VTABLE();

      struct SSensor {
         CPositionalEntity* Position;
         CVector3 Offset;
         CVector3 Direction;
         SSensor(CPositionalEntity* pc_positional_entity,
                 const CVector3& c_offset,
                 const CVector3& c_direction,
                 Real f_range) :
            Position(pc_positional_entity),
            Offset(c_offset),
            Direction(c_direction) {
            Direction.Normalize();
            Direction *= f_range;
         }
      };

      typedef std::vector<SSensor*> TSensors;

   public:

      CPrototypeProximitySensorEquippedEntity(CComposableEntity* pc_parent);

      CPrototypeProximitySensorEquippedEntity(CComposableEntity* pc_parent,
                                              const std::string& str_id);

      virtual ~CPrototypeProximitySensorEquippedEntity();

      virtual void Init(TConfigurationNode& t_tree);

      virtual std::string GetTypeDescription() const {
         return "prototype_proximity_sensors";
      }

      inline size_t GetNumSensors() const {
         return m_tSensors.size();
      }

      inline SSensor& GetSensor(size_t un_idx) {
         return *m_tSensors[un_idx];
      }

      inline TSensors& GetSensors() {
         return m_tSensors;
      }

      void AddSensor(CPositionalEntity* pc_position,
                     const CVector3& c_offset,
                     const CVector3& c_direction,
                     Real f_range);

      void AddSensorRing(CPositionalEntity* pc_position,
                         const CVector3& c_center,
                         Real f_radius,
                         const CRadians& c_start_angle,
                         Real f_range,
                         UInt32 un_num_sensors);

   protected:

      /** The list of sensors */
      TSensors m_tSensors;

   };

}

#endif
