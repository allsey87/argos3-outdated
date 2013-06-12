/**
 * @file <argos3/plugins/robots/foot-bot/simulator/footbot_proximity_default_sensor.cpp>
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 */

#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/plugins/simulator/entities/proximity_sensor_equipped_entity.h>
#include <argos3/plugins/robots/robot/simulator/body_entity.h>

#include "robot_proximity_default_sensor.h"

namespace argos {

   /****************************************/
   /****************************************/

   static CRange<Real> UNIT(0.0f, 1.0f);

   class CProximitySensorImpl : public CProximityDefaultSensor {

   public:

      virtual void SetRobot(CComposableEntity& c_entity) {
         m_pcRobot = &c_entity;
         m_pcEmbodiedEntity = &(m_pcRobot->GetComponent<CEmbodiedEntity>("body"));
         m_pcControllableEntity = &(m_pcRobot->GetComponent<CControllableEntity>("controller"));
         /* Ignore the sensing robot when checking for occlusions */
         m_tIgnoreMe.insert(m_pcEmbodiedEntity);
      }

      virtual void Init(TConfigurationNode& t_tree) {
         std::string strSensorEquippedEntityTarget;
         GetNodeAttribute(t_tree, "target", strSensorEquippedEntityTarget);
         m_pcProximityEntity = &(m_pcRobot->GetComponent<CProximitySensorEquippedEntity>
                                 (strSensorEquippedEntityTarget));
         /* Get a reference to the parent body on which this sensor is mounted */
         m_pcSensorBody = dynamic_cast<CBodyEntity*>(&m_pcProximityEntity->GetParent());
         m_pcProximityEntity->SetCanBeEnabledIfDisabled(true);
         m_pcProximityEntity->Enable();

         m_tReadings.resize(m_pcProximityEntity->GetNumSensors());
         CProximityDefaultSensor::Init(t_tree);
      }

      virtual void Update() {
         /* Ray used for scanning the environment for obstacles */
         CRay3 cScanningRay;
         CVector3 cRayStart, cRayEnd;
         /* Buffers to contain data about the intersection */
         CSpace::SEntityIntersectionItem<CEmbodiedEntity> sIntersection;
         /* Go through the sensors */
         for(UInt32 i = 0; i < m_tReadings.size(); ++i) {
            /* Compute ray for sensor i */
            cRayStart = m_pcProximityEntity->GetSensor(i).Offset;
            cRayStart.Rotate(m_pcSensorBody->GetPositionalEntity().GetOrientation());
            cRayStart += m_pcSensorBody->GetPositionalEntity().GetPosition();
            cRayEnd = m_pcProximityEntity->GetSensor(i).Offset;
            cRayEnd += m_pcProximityEntity->GetSensor(i).Direction;
            cRayEnd.Rotate(m_pcSensorBody->GetPositionalEntity().GetOrientation());
            cRayEnd += m_pcSensorBody->GetPositionalEntity().GetPosition();
            cScanningRay.Set(cRayStart,cRayEnd);
            /* Compute reading */
            /* Get the closest intersection */
            if(m_cSpace.GetClosestEmbodiedEntityIntersectedByRay(sIntersection,
                                                                 cScanningRay,
                                                                 m_tIgnoreMe)) {
               /* There is an intersection */
               if(m_bShowRays) {
                  m_pcControllableEntity->AddIntersectionPoint(cScanningRay,
                                                            sIntersection.TOnRay);
                  m_pcControllableEntity->AddCheckedRay(true, cScanningRay);
               }
               m_tReadings[i] = CalculateReading(cScanningRay.GetDistance(sIntersection.TOnRay));
            }
            else {
               /* No intersection */
               m_tReadings[i] = 0.0f;
               if(m_bShowRays) {
                  m_pcControllableEntity->AddCheckedRay(false, cScanningRay);
               }
            }
            /* Apply noise to the sensor */
            if(m_bAddNoise) {
               m_tReadings[i] += m_pcRNG->Uniform(m_cNoiseRange);
            }
            /* Trunc the reading between 0 and 1 */
         UNIT.TruncValue(m_tReadings[i]);
         }
      }

   protected:
      CComposableEntity* m_pcRobot;
      CBodyEntity* m_pcSensorBody;
      
   };

   /****************************************/
   /****************************************/

   CRobotProximityDefaultSensor::CRobotProximityDefaultSensor() :
      m_pcProximityImpl(new CProximitySensorImpl()) {}

   /****************************************/
   /****************************************/

   CRobotProximityDefaultSensor::~CRobotProximityDefaultSensor() {
      delete m_pcProximityImpl;
   }

   /****************************************/
   /****************************************/
   
    void CRobotProximityDefaultSensor::SetRobot(CComposableEntity& c_entity) {
       m_pcProximityImpl->SetRobot(c_entity);
    }

   /****************************************/
   /****************************************/

   void CRobotProximityDefaultSensor::Init(TConfigurationNode& t_tree) {
      m_pcProximityImpl->Init(t_tree);

      m_tReadings.resize(m_pcProximityImpl->GetReadings().size());
      for(UInt32 i = 0; i < m_tReadings.size(); ++i) {
         m_tReadings[i].SensorOffset = CVector3();
         m_tReadings[i].SensorDirection = CVector3();
         /*
         m_tReadings[i].SensorOffset = m_pcProximityEntity->GetSensor(i).Offset;
         m_tReadings[i].SensorDirection = m_pcProximityEntity->GetSensor(i).Direction; 
         */
         m_tReadings[i].Value = 0.0f;
      }
   }

   /****************************************/
   /****************************************/

   void CRobotProximityDefaultSensor::Update() {
      m_pcProximityImpl->Update();

      /* transfer readings into local vector */
      for(size_t i = 0; i < m_pcProximityImpl->GetReadings().size(); ++i) {
         m_tReadings[i].Value = m_pcProximityImpl->GetReadings()[i];
      }
   }

   /****************************************/
   /****************************************/

   void CRobotProximityDefaultSensor::Reset() {
      m_pcProximityImpl->Reset();
   }

   /****************************************/
   /****************************************/

   REGISTER_SENSOR(CRobotProximityDefaultSensor,
                   "robot_proximity", "default",
                   "Michael [allsey87@gmail.com]",
                   "1.0",
                   "The prototype proximity sensor",
                   "This sensor accesses a prototype's proximity sensor group. For a complete description\n"
                   "of its usage, refer to the ci_robot_proximity_sensor.h interface. For the XML\n"
                   "configuration, refer to the default proximity sensor.\n",
                   "Usable"
		  );

}
