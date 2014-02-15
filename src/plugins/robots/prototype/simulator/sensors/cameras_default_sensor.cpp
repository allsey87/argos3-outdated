/**
 * @file <argos3/plugins/robots/prototype/simulator/sensors/cameras_default_sensor.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "cameras_default_sensor.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/plugins/robots/prototype/simulator/entities/camera_equipped_entity.h>

namespace argos {

   /****************************************/
   /****************************************/

   CCamerasDefaultSensor::CCamerasDefaultSensor() :
      m_bEnabled(false),
      m_pcCamerasEntity(NULL),
      m_pcControllableEntity(NULL) {}

   /****************************************/
   /****************************************/

   CCamerasDefaultSensor::~CCamerasDefaultSensor() {
   }

   /****************************************/
   /****************************************/

   void CCamerasDefaultSensor::SetRobot(CComposableEntity& c_entity) {
      /* Get and enable omndirectional camera equipped entity */
      m_pcCameraEquippedEntity = &(c_entity.GetComponent<CCameraEquippedEntity>("camera_equipped_entity"));
      m_pcCamerasEquippedEntity->SetCanBeEnabledIfDisabled(true);
      /* Create a readings list for each camera in the container */
      m_tReadings.resize(m_pcCamerasEntity->GetAllCameras().size());
      /* Populate the descriptors for each camera */
      for(size_t i = 0; i < m_pcCamerasEntity->GetAllCameras().size(); ++i) {
         m_tDescriptors.push_back(SDescriptor(m_pcCamerasEntity->GetCamera(i).GetId(),
                                              m_pcCamerasEntity->GetCamera(i).GetHorizontalResolution(),
                                              m_pcCamerasEntity->GetCamera(i).GetVerticalResolution(),
                                              m_pcCamerasEntity->GetCamera(i).IsEnabled()));
      }
      /* Initialise the viewport vector to the correct size */
      m_vecViewports.resize(m_pcCamerasEntity->GetAllCameras().size());
      /* Create the algorithms vector for the cameras */
      m_vecAlgorithms.resize(m_pcCamerasEntity->GetAllCameras().size());
      /* Get controllable entity */
      m_pcControllableEntity = &(c_entity.GetComponent<CControllableEntity>("controller"));
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
             ++Algorithm) {
            std::string strCamera;
            GetNodeAttribute(*itAlgorithm, "camera", strCamera);
            /* find a descriptor that matches the id of the camera */
            size_t i;
            for(size_t i = 0; i < m_pcCamerasEntity->GetAllCameras().size(); ++i) {
               if(m_pcCamerasEntity->GetCamera(i).GetId() == strCamera) break;
            }
            if(i < m_pcCamerasEntity->GetAllCameras().size()) {
               CCameraSensorAlgorithm* pcAlgorithm = CFactory<CCameraSensorAlgorithm>::New(itAlgorithm->Value());
               pcAlgorithm->Init(*itPlugin);
               pcAlgorithm->SetCamera(m_pcCamerasEntity->GetCamera(i));
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
      for(size_t i = 0; i < m_pcCamerasEntity->GetAllCameras().size(); ++i) {
         /* update the viewport */
         Real fSinHalfFieldOfView = Sin(m_pcCamerasEntity->GetCamera(i).GetFieldOfView() * 0.5f);
         Real fViewingSphereRadius = (m_pcCamerasEntity->GetCamera(i).GetRange() * fSinHalfFieldOfView) / (fSinHalfFieldOfView + 1.0f);   
         /* Calculate the viewport position (center) */
         m_vecViewports[i].Position = CVector3(0.0f, 0.0f, m_pcCamerasEntity->GetCamera(i).GetRange() - fViewingSphereRadius);
         m_vecViewports[i].Position.Rotate(m_pcCamerasEntity->GetOffsetOrientation(i));
         m_vecViewports[i].Position += m_pcCamerasEntity->GetOffsetPosition(i);
         m_vecViewports[i].Position.Rotate(m_pcCamerasEntity->GetPositionalEntity(i).GetOrientation());
         m_vecViewports[i].Position += m_pcCamerasEntity->GetPositionalEntity(i).GetPosition();
         /* Calculate the viewport orientation - zero at the moment, holds with equal extents only */
         m_vecViewports[i].Orientation = CQuaternion(); 
         /* Calculate the viewport half extents */
         m_vecViewports[i].HalfExtents = CVector3(fViewingSphereRadius, fViewingSphereRadius, fViewingSphereRadius);
         /* Calculate camera position in GCS */
         m_vecViewports[i].CameraLocation = m_pcCamerasEntity->GetOffsetPosition(i);
         m_vecViewports[i].CameraLocation.Rotate(m_pcCamerasEntity->GetPositionalEntity(i).GetOrientation());
         m_vecViewports[i].CameraLocation += m_pcCamerasEntity->GetPositionalEntity(i).GetPosition();


         /* run the algorithms associated with this camera */
         for(size_t j = 0; j < m_vecAlgorithms[i].size(); ++j) {
            m_vecAlgorithms[i][j]->SetViewport(m_vecViewports[i]);
            m_tReadings[i][j] = m_vecAlgorithms[i][j]->Update();
         }

         

      
      // for(size_t i = 0; i < m_pcCamerasEntity->GetAllCameras().size(); ++i) {
         /* Calculate camera parameters */
         /* Setup up the operation for camera i */

         

      //    m_pcOperation->Setup(m_tReadings[i],
      //                         m_vecViewports[i],
      //                         cCameraPosition,
      //                         m_pcCamerasEntity->GetPositionalEntity(i).GetPosition(),
      //                         m_pcCamerasEntity->GetPositionalEntity(i).GetOrientation(),
      //                         m_pcCamerasEntity->GetOffsetPosition(i),
      //                         m_pcCamerasEntity->GetOffsetOrientation(i),
      //                         m_pcCamerasEntity->GetCamera(i).GetRoll(),
      //                         m_pcCamerasEntity->GetCamera(i).GetHorizontalResolution(),
      //                         m_pcCamerasEntity->GetCamera(i).GetVerticalResolution());
      //    /* Run the LED check operation over all entities found in the index bounded or intersected by a box */
      //    m_pcLEDIndex->ForEntitiesInBoxRange(m_vecViewports[i].Position, m_vecViewports[i].HalfExtents, *m_pcOperation);
      // }
   }

   /****************************************/
   /****************************************/

   void CCamerasDefaultSensor::Reset() {
   }

   /****************************************/
   /****************************************/

   void CCamerasDefaultSensor::Destroy() {
      delete m_pcOperation;
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
                   "A generic camera sensor to detect LEDs.",
                   "TODO\n\n",
                   "Usable"
		  );

}
