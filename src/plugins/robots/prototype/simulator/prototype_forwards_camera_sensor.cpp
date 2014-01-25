/**
 * @file <argos3/plugins/robot/prototype/simulator/prototype_colored_blob_forwards_camera_sensor.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "prototype_forwards_camera_sensor.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/positional_indices/positional_index.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/plugins/simulator/entities/led_entity.h>
#include <argos3/plugins/simulator/media/led_medium.h>

#include <argos3/plugins/robots/prototype/simulator/forwards_camera_equipped_entity.h>

namespace argos {

   /****************************************/
   /****************************************/

   class CLEDCheckOperation : public CPositionalIndex<CLEDEntity>::COperation {

   public:

      CLEDCheckOperation(//CForwardsCameraEquippedEntity& c_forwards_cam_entity,
                         CEmbodiedEntity& c_embodied_entity,
                         CControllableEntity& c_controllable_entity,
                         bool b_show_rays) :
         //         m_cForwardsCamEntity(c_forwards_cam_entity),
         m_cEmbodiedEntity(c_embodied_entity),
         m_cControllableEntity(c_controllable_entity),
         m_bShowRays(b_show_rays) {
      }

      virtual ~CLEDCheckOperation() {
         m_pcResults->ObservationList.clear();
      }

      virtual bool operator()(CLEDEntity& c_led) {
         fprintf(stderr,
                 "\nexcuting check operation on %s\n",
                 (c_led.GetContext() + c_led.GetId()).c_str());
         fprintf(stderr,
                 "\tdistance from sphere center = %.3f\n",
                 (c_led.GetPosition() - m_cViewingSpherePos).Length());

         if(c_led.GetColor() != CColor::BLACK) {
            if((c_led.GetPosition() - m_cViewingSpherePos).Length() < m_fViewingSphereRadius) {
               m_cOcclusionCheckRay.SetEnd(c_led.GetPosition());         

               if(!GetClosestEmbodiedEntityIntersectedByRay(m_sIntersectionItem,
                                                            m_cOcclusionCheckRay
                                                            //, m_cEmbodiedEntity)) {
                                                            )) {
                  //calculate the position in the image
                  
                  CQuaternion cBetweenVectors(m_cCameraPos - m_cViewingSpherePos,
                                              m_cCameraPos - c_led.GetPosition());

                  CRadians cZ, cY, cX;
                  cBetweenVectors.ToEulerAngles(cZ,cY,cX);

                  fprintf(stderr, "%s\n", (c_led.GetContext() + c_led.GetId()).c_str());
                  fprintf(stderr,
                          "\tled position = [%.3f, %.3f, %.3f]\n",
                          c_led.GetPosition().GetX(),
                          c_led.GetPosition().GetY(),
                          c_led.GetPosition().GetZ());
                  fprintf(stderr,
                          "\tangles = [%.3f, %.3f, %.3f]\n",
                          ToDegrees(cZ).GetValue(),
                          ToDegrees(cY).GetValue(),
                          ToDegrees(cX).GetValue());

                  //end

                  m_pcResults->ObservationList.push_back(
                     CCI_PrototypeForwardsCameraSensor::SObservation(c_led.GetColor(), 0u, 0u));
                  if(m_bShowRays) {
                     m_cControllableEntity.AddCheckedRay(false, CRay3(m_cCameraPos, c_led.GetPosition()));
                  }
               }
               else {
                  m_cControllableEntity.AddCheckedRay(true, CRay3(m_cCameraPos, c_led.GetPosition()));
               }
            }
         }
         return true;
      }

      void Setup(const CVector3& c_camera_pos,
                 const CVector3& c_viewing_sphere_pos,
                 Real f_viewing_sphere_radius,
                 CCI_PrototypeForwardsCameraSensor::SReading& s_results) {

         m_pcResults = &s_results;

         /* delete the previous observations */
         m_pcResults->ObservationList.clear();
         
         m_cCameraPos = c_camera_pos;
         m_cViewingSpherePos = c_viewing_sphere_pos;
         m_fViewingSphereRadius = f_viewing_sphere_radius;
         m_cOcclusionCheckRay.SetStart(m_cCameraPos);

         // Add a ray from the camera to the sphere centre for debugging
         m_cControllableEntity.AddCheckedRay(true, CRay3(m_cCameraPos, c_viewing_sphere_pos));



      }
      
   private:
      
      CCI_PrototypeForwardsCameraSensor::SReading* m_pcResults;
      CCI_PrototypeForwardsCameraSensor::SObservation::TList::iterator m_itNextResult;
      CEmbodiedEntity& m_cEmbodiedEntity;
      CControllableEntity& m_cControllableEntity;
      bool m_bShowRays;
      CVector3 m_cCameraPos;
      CVector3 m_cViewingSpherePos;
      Real m_fViewingSphereRadius;
      SEmbodiedEntityIntersectionItem m_sIntersectionItem;
      CRay3 m_cOcclusionCheckRay;
   };

   /****************************************/
   /****************************************/

   CPrototypeForwardsCameraSensor::CPrototypeForwardsCameraSensor() :
      m_bEnabled(false),
      m_pcForwardsCamerasEntity(NULL),
      m_pcControllableEntity(NULL),
      m_pcEmbodiedEntity(NULL),
      m_pcLEDIndex(NULL),
      m_pcEmbodiedIndex(NULL),
      m_fDistanceNoiseStdDev(0.0f),
      m_pcRNG(NULL),
      m_bShowRays(false) {}

   /****************************************/
   /****************************************/

   CPrototypeForwardsCameraSensor::~CPrototypeForwardsCameraSensor() {
   }

   /****************************************/
   /****************************************/

   void CPrototypeForwardsCameraSensor::SetRobot(CComposableEntity& c_entity) {
      /* Get and enable omndirectional camera equipped entity */
      m_pcForwardsCamerasEntity = &(c_entity.GetComponent<CForwardsCameraEquippedEntity>("forwards_camera_container"));
      m_pcForwardsCamerasEntity->SetCanBeEnabledIfDisabled(true);

      /* Create a readings list for each camera in the container */
      m_tReadings.resize(m_pcForwardsCamerasEntity->GetAllForwardsCameras().size());
      
      for(size_t i = 0; i < m_pcForwardsCamerasEntity->GetAllForwardsCameras().size(); ++i) {
         m_tDescriptors.push_back(SDescriptor(m_pcForwardsCamerasEntity->GetForwardsCamera(i).GetId(),
                                              m_pcForwardsCamerasEntity->GetForwardsCamera(i).GetHorizontalResolution(),
                                              m_pcForwardsCamerasEntity->GetForwardsCamera(i).GetVerticalResolution(),
                                              m_pcForwardsCamerasEntity->GetForwardsCamera(i).IsEnabled()));
      }
      /* Get controllable entity */
      m_pcControllableEntity = &(c_entity.GetComponent<CControllableEntity>("controller"));
      /* Get embodied entity */
      m_pcEmbodiedEntity = &(c_entity.GetComponent<CEmbodiedEntity>("body"));
   }

   /****************************************/
   /****************************************/

   void CPrototypeForwardsCameraSensor::Init(TConfigurationNode& t_tree) {
      try {
         /* Parent class init */
         CCI_PrototypeForwardsCameraSensor::Init(t_tree);
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
         m_pcOperation = new CLEDCheckOperation(//m_sReadings.BlobList,
                                                //*m_pcForwardsCamerasEntity,
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

   void CPrototypeForwardsCameraSensor::Update() {
      for(UInt32 i = 0;
          i < m_pcForwardsCamerasEntity->GetAllForwardsCameras().size();
          ++i) {
         
         const CForwardsCameraEntity& cForwardsCamera =
            m_pcForwardsCamerasEntity->GetForwardsCamera(i);

         if(cForwardsCamera.IsEnabled()) {
            const CQuaternion& cFowardsCameraOrientationOffset =
               m_pcForwardsCamerasEntity->GetOffsetOrientation(i);
            const CVector3& cFowardsCameraPositionOffset =
               m_pcForwardsCamerasEntity->GetOffsetPosition(i);
            const CQuaternion& cFowardsCameraAttachedBodyOrientation =
               m_pcForwardsCamerasEntity->GetPositionalEntity(i).GetOrientation();
            const CVector3& cFowardsCameraAttachedBodyPosition =
               m_pcForwardsCamerasEntity->GetPositionalEntity(i).GetPosition();
            const CRadians& cFieldOfView = cForwardsCamera.GetFieldOfView(); 
            Real fRange = cForwardsCamera.GetRange();

            /* Calculate camera parameters */
            Real fSinHalfFieldOfView = Sin(cFieldOfView * 0.5f);
            Real fViewingSphereRadius = (fRange * fSinHalfFieldOfView) / (fSinHalfFieldOfView + 1.0f);
            CVector3 cViewingSphereOffset(0.0f, 0.0f, fRange - fViewingSphereRadius);
            
            /* Calculate the viewing sphere position */
            CVector3 cViewingSpherePos = cViewingSphereOffset;
            cViewingSpherePos.Rotate(cFowardsCameraOrientationOffset);
            cViewingSpherePos += cFowardsCameraPositionOffset;
            cViewingSpherePos.Rotate(cFowardsCameraAttachedBodyOrientation);
            cViewingSpherePos += cFowardsCameraAttachedBodyPosition;
            /* Calculate camera position */
            CVector3 cCameraPos = cFowardsCameraPositionOffset;
            cCameraPos.Rotate(cFowardsCameraAttachedBodyOrientation);
            cCameraPos += cFowardsCameraAttachedBodyPosition;

            /* Setup up the operation for camera i */
            m_pcOperation->Setup(cCameraPos, cViewingSpherePos, fViewingSphereRadius, m_tReadings[i]);

            /* Update the viewing sphere for debug purposes */
            /* Should this be a permanently enabled feature? Implement at the entity level?? */
            m_pcForwardsCamerasEntity->GetForwardsCamera(i).SphereCenter = cViewingSpherePos;
            m_pcForwardsCamerasEntity->GetForwardsCamera(i).SphereRadius = fViewingSphereRadius;
         

            fprintf(stderr, 
                    "%s\n", 
                    (m_pcForwardsCamerasEntity->GetForwardsCamera(i).GetContext() + 
                     m_pcForwardsCamerasEntity->GetForwardsCamera(i).GetId()).c_str());
            fprintf(stderr,
                    "\tposition = [%.3f, %.3f, %.3f]\n",
                    cCameraPos.GetX(),
                    cCameraPos.GetY(),
                    cCameraPos.GetZ());
            fprintf(stderr,
                    "\tsphere position = [%.3f, %.3f, %.3f]\n",
                    cViewingSpherePos.GetX(),
                    cViewingSpherePos.GetY(),
                    cViewingSpherePos.GetZ());
            fprintf(stderr,
                    "\tsphere radius = [%.3f]\n",
                    fViewingSphereRadius);
            
            /*
            m_pcLEDIndex->ForEntitiesInSphereRange(
               CVector3(cViewingSpherePos.GetX(),
                        cViewingSpherePos.GetY(),
                        cViewingSpherePos.GetZ()),
               fViewingSphereRadius,
               *m_pcOperation);
            */
            
            m_pcLEDIndex->ForEntitiesInBoxRange(
               CVector3(cViewingSpherePos.GetX(),
                        cViewingSpherePos.GetY(),
                        cViewingSpherePos.GetZ()),
               CVector3(fViewingSphereRadius,
                        fViewingSphereRadius,
                        fViewingSphereRadius),
               *m_pcOperation);            
            //m_pcLEDIndex->ForAllEntities(*m_pcOperation);

         }
      }
   }

   /****************************************/
   /****************************************/

   void CPrototypeForwardsCameraSensor::Reset() {
            //      for(size_t i = 0; m_tReadings.size(); ++i) {
            

            //}
   }

   /****************************************/
   /****************************************/

   void CPrototypeForwardsCameraSensor::Destroy() {
      delete m_pcOperation;
   }

   size_t CPrototypeForwardsCameraSensor::GetNumberForwardsCameras() {
      return m_pcForwardsCamerasEntity->GetAllForwardsCameras().size();
   }


   /****************************************/
   /****************************************/

   void CPrototypeForwardsCameraSensor::Enable() {
            //m_pcForwardsCamerasEntity->Enable();
            // m_bEnabled = true;
   }

   /****************************************/
   /****************************************/

   void CPrototypeForwardsCameraSensor::Disable() {
            //      m_pcForwardsCamerasEntity->Disable();
            //      m_bEnabled = false;
   }

   /****************************************/
   /****************************************/

   REGISTER_SENSOR(CPrototypeForwardsCameraSensor,
                   "prototype_forwards_camera", "default",
                   "Michael Allwright [allsey87@gmail.com]",
                   "1.0",
                   "A generic forwards camera sensor to detect colored blobs.",
                   "TODO\n\n",
                   "Usable"
		  );

}
