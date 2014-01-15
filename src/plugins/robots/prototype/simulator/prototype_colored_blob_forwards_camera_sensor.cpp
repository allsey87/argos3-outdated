/**
 * @file <argos3/plugins/robot/prototype/simulator/prototype_colored_blob_forwards_camera_sensor.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "prototype_colored_blob_forwards_camera_sensor.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/positional_indices/positional_index.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/plugins/simulator/entities/led_entity.h>
#include <argos3/plugins/simulator/media/led_medium.h>

#include <argos3/plugins/robots/prototype/simulator/prototype_forwards_camera_equipped_entity.h>

namespace argos {

   /****************************************/
   /****************************************/

   class CLEDCheckOperation : public CPositionalIndex<CLEDEntity>::COperation {

   public:

      CLEDCheckOperation(CCI_PrototypeColoredBlobForwardsCameraSensor::TBlobList& t_blobs,
                         CPrototypeForwardsCameraEquippedEntity& c_forwards_cam_entity,
                         CEmbodiedEntity& c_embodied_entity,
                         CControllableEntity& c_controllable_entity,
                         bool b_show_rays) :
         m_tBlobs(t_blobs),
         m_cForwardsCamEntity(c_forwards_cam_entity),
         m_cEmbodiedEntity(c_embodied_entity),
         m_cControllableEntity(c_controllable_entity),
         m_bShowRays(b_show_rays) {
         m_pcRootSensingEntity = &m_cEmbodiedEntity.GetParent();
      }
      virtual ~CLEDCheckOperation() {
         while(! m_tBlobs.empty()) {
            delete m_tBlobs.back();
            m_tBlobs.pop_back();
         }
      }

      virtual bool operator()(CLEDEntity& c_led) {
         /* Process this LED only if it's lit */
         if(c_led.GetColor() != CColor::BLACK) {
            if(c_led.HasParent()) {
               /* Filter out the LEDs belonging to the sensing entity by checking if they share the same parent entity */
               m_pcRootOfLEDEntity = &c_led.GetParent();
               while(m_pcRootOfLEDEntity->HasParent()) m_pcRootOfLEDEntity = &m_pcRootOfLEDEntity->GetParent();
               if(m_pcRootSensingEntity == m_pcRootOfLEDEntity) {
                  return true;
               }
            }
            /* If we are here, it's because the LED must be processed */
            m_cOcclusionCheckRay.SetEnd(c_led.GetPosition());
            m_cLEDRelativePos = c_led.GetPosition();
            m_cLEDRelativePos -= m_cCameraPos;
            m_cLEDRelativePosXY.Set(m_cLEDRelativePos.GetX(),
                                    m_cLEDRelativePos.GetY());
            if(Abs(m_cLEDRelativePos.GetX()) < m_fGroundHalfRange &&
               Abs(m_cLEDRelativePos.GetY()) < m_fGroundHalfRange &&
               m_cLEDRelativePos.GetZ() < m_cCameraPos.GetZ() &&
               !GetClosestEmbodiedEntityIntersectedByRay(m_sIntersectionItem,
                                                         m_cOcclusionCheckRay,
                                                         m_cEmbodiedEntity)) {
               m_tBlobs.push_back(new CCI_PrototypeColoredBlobForwardsCameraSensor::SBlob(c_led.GetColor(),
                                                                                          m_cLEDRelativePosXY.Angle() - m_cCameraOrient,
                                                                                          m_cLEDRelativePosXY.Length() * 100.0f));
               if(m_bShowRays) {
                  m_cControllableEntity.AddCheckedRay(false, CRay3(m_cCameraPos, c_led.GetPosition()));
               }
            }
         }
         return true;
      }

      void Setup(Real f_ground_half_range) {
         while(! m_tBlobs.empty()) {
            delete m_tBlobs.back();
            m_tBlobs.pop_back();
         }
         m_fGroundHalfRange = f_ground_half_range;
         m_cEmbodiedEntity.GetOrientation().ToEulerAngles(m_cCameraOrient, m_cTmp1, m_cTmp2);
         m_cCameraPos = m_cEmbodiedEntity.GetPosition();
         m_cCameraPos += m_cForwardsCamEntity.GetOffset();
         m_cOcclusionCheckRay.SetStart(m_cCameraPos);
      }
      
   private:
      
      CCI_PrototypeColoredBlobForwardsCameraSensor::TBlobList& m_tBlobs;
      CPrototypeForwardsCameraEquippedEntity& m_cForwardsCamEntity;
      CEmbodiedEntity& m_cEmbodiedEntity;
      CControllableEntity& m_cControllableEntity;
      Real m_fGroundHalfRange;
      bool m_bShowRays;
      CEntity* m_pcRootSensingEntity;
      CEntity* m_pcRootOfLEDEntity;
      CVector3 m_cCameraPos;
      CRadians m_cCameraOrient;
      CRadians m_cTmp1, m_cTmp2;
      CVector3 m_cLEDRelativePos;
      CVector2 m_cLEDRelativePosXY;
      SEmbodiedEntityIntersectionItem m_sIntersectionItem;
      CRay3 m_cOcclusionCheckRay;
   };

   /****************************************/
   /****************************************/

   CPrototypeColoredBlobForwardsCameraSensor::CPrototypeColoredBlobForwardsCameraSensor() :
      m_bEnabled(false),
      m_pcForwardsCamEntity(NULL),
      m_pcControllableEntity(NULL),
      m_pcEmbodiedEntity(NULL),
      m_pcLEDIndex(NULL),
      m_pcEmbodiedIndex(NULL),
      m_fDistanceNoiseStdDev(0.0f),
      m_pcRNG(NULL),
      m_bShowRays(false) {}

   /****************************************/
   /****************************************/

   CPrototypeColoredBlobForwardsCameraSensor::~CPrototypeColoredBlobForwardsCameraSensor() {
   }

   /****************************************/
   /****************************************/

   void CPrototypeColoredBlobForwardsCameraSensor::SetRobot(CComposableEntity& c_entity) {
      /* Get and enable omndirectional camera equipped entity */
      m_pcForwardsCamEntity = &(c_entity.GetComponent<CPrototypeForwardsCameraEquippedEntity>("prototype_forwards_camera"));
      m_pcForwardsCamEntity->SetCanBeEnabledIfDisabled(true);
      /* Get controllable entity */
      m_pcControllableEntity = &(c_entity.GetComponent<CControllableEntity>("controller"));
      /* Get embodied entity */
      m_pcEmbodiedEntity = &(c_entity.GetComponent<CEmbodiedEntity>("body"));
   }

   /****************************************/
   /****************************************/

   void CPrototypeColoredBlobForwardsCameraSensor::Init(TConfigurationNode& t_tree) {
      try {
         /* Parent class init */
         CCI_PrototypeColoredBlobForwardsCameraSensor::Init(t_tree);
         /* Show rays? */
         GetNodeAttributeOrDefault(t_tree, "show_rays", m_bShowRays, m_bShowRays);
         /* Parse noise */
         GetNodeAttributeOrDefault(t_tree, "noise_std_dev", m_fDistanceNoiseStdDev, m_fDistanceNoiseStdDev);
         if(m_fDistanceNoiseStdDev > 0.0f) {
            m_pcRNG = CRandom::CreateRNG("argos");
         }
         /* Get LED medium from id specified in the XML */
         std::string strMedium;
         GetNodeAttribute(t_tree, "medium", strMedium);
         m_pcLEDIndex = &(CSimulator::GetInstance().GetMedium<CLEDMedium>(strMedium).GetIndex());
         /* Create check operation */
         m_pcOperation = new CLEDCheckOperation(m_sReadings.BlobList,
                                                *m_pcForwardsCamEntity,
                                                *m_pcEmbodiedEntity,
                                                *m_pcControllableEntity,
                                                m_bShowRays);
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Error initializing the prototype colored blob forwards camera sensor", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CPrototypeColoredBlobForwardsCameraSensor::Update() {
      if(m_bEnabled) {
         /* Increase data counter */
         ++m_sReadings.Counter;
         /* Calculate range on the ground */
         CVector3 cCameraPos = m_pcForwardsCamEntity->GetOffset();
         cCameraPos += m_pcEmbodiedEntity->GetPosition();
         Real fGroundHalfRange = cCameraPos.GetZ() * Tan(m_pcForwardsCamEntity->GetAperture());
         /* Prepare the operation */
         m_pcOperation->Setup(fGroundHalfRange);
         /* Go through LED entities in box range */
         m_pcLEDIndex->ForEntitiesInBoxRange(
            CVector3(cCameraPos.GetX(),
                     cCameraPos.GetY(),
                     cCameraPos.GetZ() * 0.5f),
            CVector3(fGroundHalfRange, fGroundHalfRange, cCameraPos.GetZ() * 0.5f),
            *m_pcOperation);
      }
   }

   /****************************************/
   /****************************************/

   void CPrototypeColoredBlobForwardsCameraSensor::Reset() {
      m_sReadings.Counter = 0;
      m_sReadings.BlobList.clear();
   }

   /****************************************/
   /****************************************/

   void CPrototypeColoredBlobForwardsCameraSensor::Destroy() {
      delete m_pcOperation;
   }

   /****************************************/
   /****************************************/

   void CPrototypeColoredBlobForwardsCameraSensor::Enable() {
      m_pcForwardsCamEntity->Enable();
      m_bEnabled = true;
   }

   /****************************************/
   /****************************************/

   void CPrototypeColoredBlobForwardsCameraSensor::Disable() {
      m_pcForwardsCamEntity->Disable();
      m_bEnabled = false;
   }

   /****************************************/
   /****************************************/

   REGISTER_SENSOR(CPrototypeColoredBlobForwardsCameraSensor,
                   "prototype_colored_blob_forwards_camera", "default",
                   "Michael Allwright [allsey87@gmail.com]",
                   "1.0",
                   "A generic forwards camera sensor to detect colored blobs.",
                   "TODO\n\n",
                   "Usable"
		  );

}
