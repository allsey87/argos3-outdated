/**
 * @file <argos3/plugins/robots/prototype/simulator/sensors/cameras_default_sensor.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "cameras_default_sensor.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/plugins/robots/prototype/simulator/entities/camera_equipped_entity.h>
#include <argos3/plugins/robots/prototype/simulator/sensors/cameras_default_sensor_algorithm.h>


namespace argos {

   /****************************************/
   /****************************************/

   CCamerasDefaultSensor::CCamerasDefaultSensor() :
      m_bEnabled(false),
      m_pcCameraEquippedEntity(NULL) {}
      //m_pcControllableEntity(NULL) {}

   /****************************************/
   /****************************************/

   CCamerasDefaultSensor::~CCamerasDefaultSensor() {
   }

   /****************************************/
   /****************************************/

   void CCamerasDefaultSensor::SetRobot(CComposableEntity& c_entity) {
      /* Get and enable omndirectional camera equipped entity */
      m_pcCameraEquippedEntity = &(c_entity.GetComponent<CCameraEquippedEntity>("camera_equipped_entity"));
      m_pcCameraEquippedEntity->SetCanBeEnabledIfDisabled(true);
      /* Create a readings list for each camera in the container */
      m_tReadings.resize(m_pcCameraEquippedEntity->GetAllCameras().size());
      /* Populate the descriptors for each camera */
      for(size_t i = 0; i < m_pcCameraEquippedEntity->GetAllCameras().size(); ++i) {
         m_tDescriptors.push_back(SDescriptor(m_pcCameraEquippedEntity->GetCamera(i).GetId(),
                                              m_pcCameraEquippedEntity->GetCamera(i).GetHorizontalResolution(),
                                              m_pcCameraEquippedEntity->GetCamera(i).GetVerticalResolution(),
                                              m_pcCameraEquippedEntity->GetCamera(i).IsEnabled()));
      }
      /* Initialise the viewport vector to the correct size */
      m_vecViewports.resize(m_pcCameraEquippedEntity->GetAllCameras().size());
      /* Create the algorithms vector for the cameras */
      m_vecAlgorithms.resize(m_pcCameraEquippedEntity->GetAllCameras().size());
      /* Get controllable entity */
      //m_pcControllableEntity = &(c_entity.GetComponent<CControllableEntity>("controller"));
   }

   /****************************************/
   /****************************************/

   void CCamerasDefaultSensor::Init(TConfigurationNode& t_tree) {
      try {
         /* Parent class init */
         CCI_CamerasSensor::Init(t_tree);

         TConfigurationNodeIterator itAlgorithm;
         for(itAlgorithm = itAlgorithm.begin(&GetNode(t_tree, "algorithms"));
             itAlgorithm != itAlgorithm.end();
             ++itAlgorithm) {
            std::string strCamera;
            GetNodeAttribute(*itAlgorithm, "camera", strCamera);
            /* find a descriptor that matches the id of the camera */
            size_t i = 0;
            for(size_t i = 0; i < m_pcCameraEquippedEntity->GetAllCameras().size(); ++i) {
               if(m_pcCameraEquippedEntity->GetCamera(i).GetId() == strCamera) break;
            }
            if(i < m_pcCameraEquippedEntity->GetAllCameras().size()) {
               CCamerasDefaultSensorAlgorithm* pcAlgorithm = 
                  CFactory<CCamerasDefaultSensorAlgorithm>::New(itAlgorithm->Value());
               pcAlgorithm->SetCamera(*m_pcCameraEquippedEntity, i);
               pcAlgorithm->Init(*itAlgorithm);
               m_vecAlgorithms[i].push_back(pcAlgorithm);
            }
            else {
               THROW_ARGOSEXCEPTION("Could not assign algorithm \"" << itAlgorithm->Value() <<
                                    "\" to camera " << strCamera << "\". Camera does not exist.");
            }
         }
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Error initializing the camera sensor", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CCamerasDefaultSensor::Update() {
      for(size_t i = 0; i < m_pcCameraEquippedEntity->GetAllCameras().size(); ++i) {
         /* update the viewport */
         Real fSinHalfFieldOfView = Sin(m_pcCameraEquippedEntity->GetCamera(i).GetFieldOfView() * 0.5f);
         Real fViewingSphereRadius = (m_pcCameraEquippedEntity->GetCamera(i).GetRange() * fSinHalfFieldOfView) / (fSinHalfFieldOfView + 1.0f);   
         /* Calculate the viewport position (center) */
         m_vecViewports[i].Position = CVector3(0.0f, 0.0f, m_pcCameraEquippedEntity->GetCamera(i).GetRange() - fViewingSphereRadius);
         m_vecViewports[i].Position.Rotate(m_pcCameraEquippedEntity->GetOffsetOrientation(i));
         m_vecViewports[i].Position += m_pcCameraEquippedEntity->GetOffsetPosition(i);
         m_vecViewports[i].Position.Rotate(m_pcCameraEquippedEntity->GetPositionalEntity(i).GetOrientation());
         m_vecViewports[i].Position += m_pcCameraEquippedEntity->GetPositionalEntity(i).GetPosition();
         /* Calculate the viewport orientation - zero at the moment, holds with equal extents only */
         m_vecViewports[i].Orientation = CQuaternion(); 
         /* Calculate the viewport half extents */
         m_vecViewports[i].HalfExtents = CVector3(fViewingSphereRadius, fViewingSphereRadius, fViewingSphereRadius);
         /* Calculate camera position in GCS */
         m_vecViewports[i].CameraLocation = m_pcCameraEquippedEntity->GetOffsetPosition(i);
         m_vecViewports[i].CameraLocation.Rotate(m_pcCameraEquippedEntity->GetPositionalEntity(i).GetOrientation());
         m_vecViewports[i].CameraLocation += m_pcCameraEquippedEntity->GetPositionalEntity(i).GetPosition();


         /* run the algorithms associated with this camera */
         for(size_t j = 0; j < m_vecAlgorithms[i].size(); ++j) {
            m_vecAlgorithms[i][j]->SetViewport(m_vecViewports[i]);
            //m_tReadings[i][j] = m_vecAlgorithms[i][j]->Update();
            m_vecAlgorithms[i][j]->Update();
            //m_vecAlgorithms[i][j]->WriteResultsToTable(); ??
         }
      }
   }

   /****************************************/
   /****************************************/

   void CCamerasDefaultSensor::Reset() {
   }

   /****************************************/
   /****************************************/

   void CCamerasDefaultSensor::Destroy() {
   }

   /****************************************/
   /****************************************/

   void CCamerasDefaultSensor::Enable() {
   }

   /****************************************/
   /****************************************/

   void CCamerasDefaultSensor::Disable() {
   }

   /****************************************/
   /****************************************/

   REGISTER_SENSOR(CCamerasDefaultSensor,
                   "cameras", "default",
                   "Michael Allwright [allsey87@gmail.com]",
                   "1.0",
                   "A generic camera sensor to interface the camera entity.",
                   "A generic camera sensor to interface the camera entity that\n"
                   "is capable of running multiple image processing algorithms",
                   "Underdevelopment"
		  );

}
