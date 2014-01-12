/**
 * @file <argos3/plugins/robots/prototype/simulator/prototype_proximity_default_sensor.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/plugins/robots/prototype/simulator/prototype_proximity_sensor_equipped_entity.h>
#include <argos3/plugins/robots/prototype/simulator/body_entity.h>

#include "prototype_proximity_default_sensor.h"

namespace argos {

   /****************************************/
   /****************************************/

   static CRange<Real> UNIT(0.0f, 1.0f);

   /****************************************/
   /****************************************/

   CPrototypeProximityDefaultSensor::CPrototypeProximityDefaultSensor() {}

   /****************************************/
   /****************************************/

   CPrototypeProximityDefaultSensor::~CPrototypeProximityDefaultSensor() {}

   /****************************************/
   /****************************************/
   
    void CPrototypeProximityDefaultSensor::SetRobot(CComposableEntity& c_entity) {
       m_pcEmbodiedEntity = &(c_entity.GetComponent<CEmbodiedEntity>("body"));
       m_pcControllableEntity = &(c_entity.GetComponent<CControllableEntity>("controller"));
       m_pcPrototypeProximityEntity = &(c_entity.GetComponent<CPrototypeProximitySensorEquippedEntity>("prototype_proximity_sensors"));
       m_pcPrototypeProximityEntity->SetCanBeEnabledIfDisabled(true);
       m_pcPrototypeProximityEntity->Enable();
    }

   /****************************************/
   /****************************************/

   void CPrototypeProximityDefaultSensor::Init(TConfigurationNode& t_tree) {
      try {
         CCI_PrototypeProximitySensor::Init(t_tree);
         /* Show rays? */
         GetNodeAttributeOrDefault(t_tree, "show_rays", m_bShowRays, m_bShowRays);
         /* Parse noise level */
         Real fNoiseLevel = 0.0f;
         GetNodeAttributeOrDefault(t_tree, "noise_level", fNoiseLevel, fNoiseLevel);
         if(fNoiseLevel < 0.0f) {
            THROW_ARGOSEXCEPTION("Can't specify a negative value for the noise level of the proximity sensor");
         }
         else if(fNoiseLevel > 0.0f) {
            m_bAddNoise = true;
            m_cNoiseRange.Set(-fNoiseLevel, fNoiseLevel);
            m_pcRNG = CRandom::CreateRNG("argos");
         }
         m_tReadings.resize(m_pcPrototypeProximityEntity->GetNumSensors());
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Initialization error in prototype default proximity sensor", ex);
      }

      for(UInt32 i = 0; i < m_tReadings.size(); ++i) {
         /*
         m_tReadings[i].SensorOffset = CVector3();
         m_tReadings[i].SensorDirection = CVector3();
         */
         m_tReadings[i].SensorOffset = m_pcPrototypeProximityEntity->GetSensor(i).Offset;
         m_tReadings[i].SensorDirection = m_pcPrototypeProximityEntity->GetSensor(i).Direction; 
         m_tReadings[i].Value = 0.0f;
      }
   }

   /****************************************/
   /****************************************/

   void CPrototypeProximityDefaultSensor::Update() {
      /* Ray used for scanning the environment for obstacles */
      CRay3 cScanningRay;
      CVector3 cRayStart, cRayEnd;
      /* Buffers to contain data about the intersection */
      SEmbodiedEntityIntersectionItem sIntersection;
      /* Go through the sensors */
      for(UInt32 i = 0; i < m_tReadings.size(); ++i) {        
         /* Compute ray for sensor i */
         cRayStart = m_pcPrototypeProximityEntity->GetSensor(i).Offset;
         cRayStart.Rotate(m_pcPrototypeProximityEntity->GetSensor(i).Position->GetOrientation());
         cRayStart += m_pcPrototypeProximityEntity->GetSensor(i).Position->GetPosition();
         
         cRayEnd = m_pcPrototypeProximityEntity->GetSensor(i).Offset;
         cRayEnd += m_pcPrototypeProximityEntity->GetSensor(i).Direction;
         cRayEnd.Rotate(m_pcPrototypeProximityEntity->GetSensor(i).Position->GetOrientation());
         cRayEnd += m_pcPrototypeProximityEntity->GetSensor(i).Position->GetPosition();
         
         cScanningRay.Set(cRayStart,cRayEnd);
         /* Compute reading */
         /* Get the closest intersection */
         if(GetClosestEmbodiedEntityIntersectedByRay(sIntersection,
                                                     cScanningRay,
                                                     *m_pcEmbodiedEntity)) {
            /* There is an intersection */
            if(m_bShowRays) {
               m_pcControllableEntity->AddIntersectionPoint(cScanningRay,
                                                            sIntersection.TOnRay);
               m_pcControllableEntity->AddCheckedRay(true, cScanningRay);
            }
            m_tReadings[i].Value = CalculateReading(cScanningRay.GetDistance(sIntersection.TOnRay));
         }
         else {
            /* No intersection */
            m_tReadings[i].Value = 0.0f;
            if(m_bShowRays) {
               m_pcControllableEntity->AddCheckedRay(false, cScanningRay);
            }
         }
         /* Apply noise to the sensor */
         if(m_bAddNoise) {
            m_tReadings[i].Value += m_pcRNG->Uniform(m_cNoiseRange);
         }
         /* Trunc the reading between 0 and 1 */
         UNIT.TruncValue(m_tReadings[i].Value);
      }
   }

   /****************************************/
   /****************************************/

   void CPrototypeProximityDefaultSensor::Reset() {
      for(UInt32 i = 0; i < GetReadings().size(); ++i) {
         m_tReadings[i].Value = 0.0f;
      }
   }

   /****************************************/
   /****************************************/

   Real CPrototypeProximityDefaultSensor::CalculateReading(Real f_distance) {
      return Exp(-f_distance);
   }

   /****************************************/
   /****************************************/

   REGISTER_SENSOR(CPrototypeProximityDefaultSensor,
                   "prototype_proximity", "default",
                   "Michael [allsey87@gmail.com]",
                   "1.0",
                   "The prototype proximity sensor",
                   "This sensor accesses a prototype's proximity sensor group. For a complete description\n"
                   "of its usage, refer to the ci_prototype_proximity_sensor.h interface. For the XML\n"
                   "configuration, refer to the default proximity sensor.\n",
                   "Under development"
                   );
}
