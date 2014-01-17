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
      }
      virtual ~CLEDCheckOperation() {
         while(! m_tBlobs.empty()) {
            delete m_tBlobs.back();
            m_tBlobs.pop_back();
         }
      }

      virtual bool operator()(CLEDEntity& c_led) {
         /* Process this LED only if it's lit */
         /*
         fprintf(stderr,
                 "\tchecking LED %s at [%.3f, %.3f, %.3f]\n",
                 (c_led.GetContext() + c_led.GetId()).c_str(),
                 c_led.GetPosition().GetX(),
                 c_led.GetPosition().GetY(),
                 c_led.GetPosition().GetZ());
         */
         if(c_led.GetColor() != CColor::BLACK) {
            if((c_led.GetPosition() - m_cViewingSpherePos).Length() < m_fViewingSphereRadius) {
               m_cOcclusionCheckRay.SetEnd(c_led.GetPosition());         
               if(!GetClosestEmbodiedEntityIntersectedByRay(m_sIntersectionItem,
                                                            m_cOcclusionCheckRay,
                                                            m_cEmbodiedEntity)) {
                  //fprintf(stderr, "\tprocessed!\n");
                  m_tBlobs.push_back(new CCI_PrototypeColoredBlobForwardsCameraSensor::SBlob(c_led.GetColor(),
                                                                                             CRadians::ZERO,
                                                                                             0.0f));
                  if(m_bShowRays) {
                     m_cControllableEntity.AddCheckedRay(false, CRay3(m_cCameraPos, c_led.GetPosition()));
                  }
               }
            }   
         }
         return true;
      }

      void Setup(const CVector3& c_camera_pos,
                 const CVector3& c_viewing_sphere_pos,
                 Real f_viewing_sphere_radius) {
         while(! m_tBlobs.empty()) {
            delete m_tBlobs.back();
            m_tBlobs.pop_back();
         }
         
         m_cCameraPos = c_camera_pos;
         m_cViewingSpherePos = c_viewing_sphere_pos;
         m_fViewingSphereRadius = f_viewing_sphere_radius;
         m_cOcclusionCheckRay.SetStart(m_cCameraPos);
         /*
         fprintf(stderr,
                 "Setup of camera complete! Rays start at [%.3f, %.3f, %.3f]\n",
                 m_cOcclusionCheckRay.GetStart().GetX(),
                 m_cOcclusionCheckRay.GetStart().GetY(),
                 m_cOcclusionCheckRay.GetStart().GetZ());
         */
      }
      
   private:
      
      CCI_PrototypeColoredBlobForwardsCameraSensor::TBlobList& m_tBlobs;
      CPrototypeForwardsCameraEquippedEntity& m_cForwardsCamEntity;
      CEmbodiedEntity& m_cEmbodiedEntity;
      CControllableEntity& m_cControllableEntity;
      bool m_bShowRays;
      CVector3 m_cCameraPos;
      CVector3 m_cViewingSpherePos;
      Real m_fViewingSphereRadius;
      //   CRadians m_cCameraOrient;
      //   CRadians m_cTmp1, m_cTmp2;
      //      CVector3 m_cLEDRelativePos;
      //      CVector2 m_cLEDRelativePosXY;
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
         
         const Real fRange = m_pcForwardsCamEntity->GetRange(); 
         const CRadians& cFieldOfView = m_pcForwardsCamEntity->GetFieldOfView(); 

         Real fSinHalfFieldOfView = Sin(cFieldOfView * 0.5f);
         Real fViewingSphereRadius = (fRange * fSinHalfFieldOfView) / (fSinHalfFieldOfView + 1.0f);
         CVector3 cViewingSphereOffset(0.0f, 0.0f, fRange - fViewingSphereRadius);

         /* Calculate the viewing sphere position */
         CVector3 cViewingSpherePos = cViewingSphereOffset;
         cViewingSpherePos.Rotate(m_pcForwardsCamEntity->GetOffsetOrientation());
         cViewingSpherePos += m_pcForwardsCamEntity->GetOffsetPosition();
         cViewingSpherePos.Rotate(m_pcForwardsCamEntity->GetPositionalEntity()->GetOrientation());
         cViewingSpherePos += m_pcForwardsCamEntity->GetPositionalEntity()->GetPosition();

         CVector3 cCameraPos = m_pcForwardsCamEntity->GetOffsetPosition();
         cCameraPos.Rotate(m_pcForwardsCamEntity->GetPositionalEntity()->GetOrientation());
         cCameraPos += m_pcForwardsCamEntity->GetPositionalEntity()->GetPosition();


         /* use the orientation to aim the camera, i.e rotate the sphere wrt front of the camera */
         /*
         fprintf(stderr, "Updating CPrototypeColoredBlobForwardsCameraSensor\n");
         fprintf(stderr, 
                 "\tAttached to: %s at [%.3f, %.3f, %.3f]\n", 
                 (m_pcForwardsCamEntity->GetPositionalEntity()->GetContext() + 
                  m_pcForwardsCamEntity->GetPositionalEntity()->GetId()).c_str(),
                 m_pcForwardsCamEntity->GetPositionalEntity()->GetPosition().GetX(),
                 m_pcForwardsCamEntity->GetPositionalEntity()->GetPosition().GetY(),
                 m_pcForwardsCamEntity->GetPositionalEntity()->GetPosition().GetZ());
         fprintf(stderr, 
                 "\tCamera is at [%.3f, %.3f, %.3f]\n",
                 cCameraPos.GetX(),
                 cCameraPos.GetY(),
                 cCameraPos.GetZ());
         */
         /* Prepare the operation */
         m_pcOperation->Setup(cCameraPos, cViewingSpherePos, fViewingSphereRadius);
         /* Go through LED entities in box range */
         

         //fprintf(stderr, "Checking LEDs\n");
         //m_pcLEDIndex->ForAllEntities(*m_pcOperation);
         m_pcForwardsCamEntity->SphereCenter = cViewingSpherePos;
         m_pcForwardsCamEntity->SphereRadius = fViewingSphereRadius;
         
         m_pcLEDIndex->ForEntitiesInSphereRange(
            CVector3(cViewingSpherePos.GetX(),
                     cViewingSpherePos.GetY(),
                     cViewingSpherePos.GetZ()),
            fViewingSphereRadius,
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
