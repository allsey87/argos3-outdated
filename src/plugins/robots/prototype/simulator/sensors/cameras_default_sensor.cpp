/**
 * @file <argos3/plugins/robots/prototype/simulator/sensors/cameras_default_sensor.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "cameras_default_sensor.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/plugins/robots/prototype/simulator/entities/camera_equipped_entity.h>
#include <argos3/plugins/robots/prototype/simulator/sensors/cameras_sensor_algorithm.h>

namespace argos {

   /****************************************/
   /****************************************/

   CCamerasDefaultSensor::CCamerasDefaultSensor() :
      m_bEnabled(false),
      m_bShowFrustums(false),
      m_pcCameraEquippedEntity(NULL),
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
      m_pcCameraEquippedEntity->SetCanBeEnabledIfDisabled(true);
      /* Initialise the fustrum and discriptor vectors to the correct length */
      m_vecData.resize(m_pcCameraEquippedEntity->GetAllCameras().size());
      m_tDescriptors.resize(m_pcCameraEquippedEntity->GetAllCameras().size());
      /* Populate the descriptors for each camera */
      for(size_t i = 0; i < m_pcCameraEquippedEntity->GetAllCameras().size(); ++i) {
         const CCameraEntity& cCamera = m_pcCameraEquippedEntity->GetCamera(i);
         /* get referenced to the current camera data and descriptor */
         CCamerasSensorSimulatedAlgorithm::SData& sData = m_vecData[i];
         SDescriptor& sDescriptor = m_tDescriptors[i];
         // statically defined - get these parameters from XML       
         sData.NearDepth = 0.050; // 5cm, focus range is 10cm
         sData.FarDepth = 0.500; // 50cm, testing
         /* calculate frustum near and far plane dimensions */
         sData.ProjectionMatrix = cCamera.GetCameraMatrix();
         Real fWidthToDepthRatio = (0.5 * cCamera.GetHorizontalResolution()) / sData.ProjectionMatrix(0,0);
         Real fHeightToDepthRatio = (0.5 * cCamera.GetVerticalResolution()) / sData.ProjectionMatrix(1,1);
         sData.NearPlaneHeight = fHeightToDepthRatio * sData.NearDepth;
         sData.NearPlaneWidth = fWidthToDepthRatio * sData.NearDepth;
         sData.FarPlaneHeight = fHeightToDepthRatio * sData.FarDepth;
         sData.FarPlaneWidth = fWidthToDepthRatio * sData.FarDepth;
         
         /* set up the camera to body transformation matrix */
         sData.CameraToBodyTransform.SetFromComponents(m_pcCameraEquippedEntity->GetOffsetOrientation(i),
                                                          m_pcCameraEquippedEntity->GetOffsetPosition(i));
         /* Populate the descriptor for the camera */
         sDescriptor.Id = cCamera.GetId();
         /* todo: clean up: some of these may no longer be required */
         sDescriptor.HorizontalResolution = cCamera.GetHorizontalResolution();
         sDescriptor.VerticalResolution = cCamera.GetVerticalResolution();
         sDescriptor.CameraMatrix = cCamera.GetCameraMatrix();
         sDescriptor.DistortionParameters = cCamera.GetDistortionParameters();
         sDescriptor.Enabled = cCamera.IsEnabled();
      }
      /* Get controllable entity */
      m_pcControllableEntity = &(c_entity.GetComponent<CControllableEntity>("controller"));
   }

   /****************************************/
   /****************************************/

   void CCamerasDefaultSensor::Init(TConfigurationNode& t_tree) {
      try {
         /* Parent class init */
         CCI_CamerasSensor::Init(t_tree);
         /* Show the frustums */
         GetNodeAttributeOrDefault(t_tree, "show_frustums",m_bShowFrustums, m_bShowFrustums);
         TConfigurationNodeIterator itAlgorithm;
         for(itAlgorithm = itAlgorithm.begin(&GetNode(t_tree, "algorithms"));
             itAlgorithm != itAlgorithm.end();
             ++itAlgorithm) {
            std::string strCamera;
            GetNodeAttribute(*itAlgorithm, "camera", strCamera);
            /* find a descriptor that matches the id of the camera */
            size_t i = 0;
            for(i = 0; i < m_pcCameraEquippedEntity->GetAllCameras().size(); ++i) {
               if(m_pcCameraEquippedEntity->GetCamera(i).GetId() == strCamera) break;
            }
            if(i < m_pcCameraEquippedEntity->GetAllCameras().size()) {
               CCamerasSensorSimulatedAlgorithm* pcAlgorithm = 
                  CFactory<CCamerasSensorSimulatedAlgorithm>::New(itAlgorithm->Value());
               CCI_CamerasSensorAlgorithm* pcCIAlgorithm = dynamic_cast<CCI_CamerasSensorAlgorithm*>(pcAlgorithm);
               if(pcCIAlgorithm == NULL) {
                  THROW_ARGOSEXCEPTION("Algorithm \"" << itAlgorithm->Value() << "\" does not inherit from CCI_CamerasSensorAlgorithm");
               }
               pcAlgorithm->SetData(m_vecData[i]);
               /* Init the control interface */
               pcCIAlgorithm->Init(*itAlgorithm);
               /* Todo: check if these maps are required */
               m_mapSimulatedAlgorithms[m_pcCameraEquippedEntity->GetCamera(i).GetId()][itAlgorithm->Value()] =
                  pcAlgorithm;
               m_mapAlgorithms[m_pcCameraEquippedEntity->GetCamera(i).GetId()][itAlgorithm->Value()] =
                  pcCIAlgorithm;
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



   /* Frustum culling based on: http://www.lighthouse3d.com/tutorials/view-frustum-culling/ */

   void CCamerasDefaultSensor::Update() {
      for(UInt32 un_cam_idx = 0; un_cam_idx < m_pcCameraEquippedEntity->GetAllCameras().size(); ++un_cam_idx) {
         const CPositionalEntity& cCameraPositionalEntity = m_pcCameraEquippedEntity->GetPositionalEntity(un_cam_idx);
         CCamerasSensorSimulatedAlgorithm::SData& sData = m_vecData[un_cam_idx];
         /* calculate transformation matrices */
         CTransformationMatrix3 cBodyToWorldTransform(cCameraPositionalEntity.GetOrientation(),
                                                      cCameraPositionalEntity.GetPosition());
         sData.CameraToWorldTransform = cBodyToWorldTransform * sData.CameraToBodyTransform;
         sData.WorldToCameraTransform = sData.CameraToWorldTransform.GetInverse();
         /* calculate camera direction vectors */
         sData.CameraLocation = sData.CameraToWorldTransform.GetTranslationVector();
         sData.LookAt = sData.CameraToWorldTransform * (CVector3::Z);
         sData.Up = sData.CameraToWorldTransform * (-CVector3::Y);
         CVector3 cZ(sData.CameraLocation - sData.LookAt);
         cZ.Normalize();        
         CVector3 cX(sData.Up);
         cX.CrossProduct(cZ);
         cX.Normalize();
         CVector3 cY(cZ);
         cY.CrossProduct(cX);
         CVector3 cNearCenter(sData.CameraLocation - cZ * sData.NearDepth);
         CVector3 cFarCenter(sData.CameraLocation - cZ * sData.FarDepth);
         /* compute plane coordinates */
         /* these coordinates are kept for debugging */
         sData.NearTopLeft = cNearCenter + (cY * sData.NearPlaneHeight) - (cX * sData.NearPlaneWidth);
         sData.NearTopRight = cNearCenter + (cY * sData.NearPlaneHeight) + (cX * sData.NearPlaneWidth);
         sData.NearBottomLeft = cNearCenter - (cY * sData.NearPlaneHeight) - (cX * sData.NearPlaneWidth);
         sData.NearBottomRight = cNearCenter - (cY * sData.NearPlaneHeight) + (cX * sData.NearPlaneWidth);
         sData.FarTopLeft = cFarCenter + (cY * sData.FarPlaneHeight) - (cX * sData.FarPlaneWidth);
         sData.FarTopRight = cFarCenter + (cY * sData.FarPlaneHeight) + (cX * sData.FarPlaneWidth);
         sData.FarBottomLeft = cFarCenter - (cY * sData.FarPlaneHeight) - (cX * sData.FarPlaneWidth);
         sData.FarBottomRight = cFarCenter - (cY * sData.FarPlaneHeight) + (cX * sData.FarPlaneWidth);
         /* show frustum */
         if(m_bShowFrustums) {
            m_pcControllableEntity->GetCheckedRays().emplace_back(true, CRay3(sData.NearTopLeft, sData.NearTopRight));
            m_pcControllableEntity->GetCheckedRays().emplace_back(true, CRay3(sData.NearTopRight, sData.NearBottomRight));
            m_pcControllableEntity->GetCheckedRays().emplace_back(true, CRay3(sData.NearBottomRight, sData.NearBottomLeft));
            m_pcControllableEntity->GetCheckedRays().emplace_back(true, CRay3(sData.NearBottomLeft, sData.NearTopLeft));
            m_pcControllableEntity->GetCheckedRays().emplace_back(true, CRay3(sData.FarTopLeft, sData.FarTopRight));
            m_pcControllableEntity->GetCheckedRays().emplace_back(true, CRay3(sData.FarTopRight, sData.FarBottomRight));
            m_pcControllableEntity->GetCheckedRays().emplace_back(true, CRay3(sData.FarBottomRight, sData.FarBottomLeft));
            m_pcControllableEntity->GetCheckedRays().emplace_back(true, CRay3(sData.FarBottomLeft, sData.FarTopLeft));
            m_pcControllableEntity->GetCheckedRays().emplace_back(true, CRay3(sData.NearTopLeft, sData.FarTopLeft));
            m_pcControllableEntity->GetCheckedRays().emplace_back(true, CRay3(sData.NearTopRight, sData.FarTopRight));
            m_pcControllableEntity->GetCheckedRays().emplace_back(true, CRay3(sData.NearBottomRight, sData.FarBottomRight));
            m_pcControllableEntity->GetCheckedRays().emplace_back(true, CRay3(sData.NearBottomLeft, sData.FarBottomLeft));
         }
         /* building bounding box */
         CVector3 cBoundingBoxMinCorner(cNearCenter), cBoundingBoxMaxCorner(cNearCenter);
         for(const CVector3& c_point : {sData.NearTopLeft, sData.NearTopRight, sData.NearBottomLeft, sData.NearBottomRight, 
                                        sData.FarTopLeft, sData.FarTopRight, sData.FarBottomLeft, sData.FarBottomRight}) {
            if(c_point.GetX() > cBoundingBoxMaxCorner.GetX()) {
               cBoundingBoxMaxCorner.SetX(c_point.GetX());
            }
            if(c_point.GetX() < cBoundingBoxMinCorner.GetX()) {
               cBoundingBoxMinCorner.SetX(c_point.GetX());
            }
            if(c_point.GetY() > cBoundingBoxMaxCorner.GetY()) {
               cBoundingBoxMaxCorner.SetY(c_point.GetY());
            }
            if(c_point.GetY() < cBoundingBoxMinCorner.GetY()) {
               cBoundingBoxMinCorner.SetY(c_point.GetY());
            }
            if(c_point.GetZ() > cBoundingBoxMaxCorner.GetZ()) {
               cBoundingBoxMaxCorner.SetZ(c_point.GetZ());
            }
            if(c_point.GetZ() < cBoundingBoxMinCorner.GetZ()) {
               cBoundingBoxMinCorner.SetZ(c_point.GetZ());
            }
         }
         cBoundingBoxMaxCorner *= 0.5;
         cBoundingBoxMinCorner *= 0.5;
         sData.BoundingBoxPosition = (cBoundingBoxMaxCorner + cBoundingBoxMinCorner);
         sData.BoundingBoxHalfExtents = (cBoundingBoxMaxCorner - cBoundingBoxMinCorner);
         /* build planes */
         sData.BoundingPlanes[0].SetFromThreePoints(sData.NearTopRight,sData.NearTopLeft,sData.FarTopLeft);
         sData.BoundingPlanes[1].SetFromThreePoints(sData.NearBottomLeft,sData.NearBottomRight,sData.FarBottomRight);
         sData.BoundingPlanes[2].SetFromThreePoints(sData.NearTopLeft,sData.NearBottomLeft,sData.FarBottomLeft);
         sData.BoundingPlanes[3].SetFromThreePoints(sData.NearBottomRight,sData.NearTopRight,sData.FarBottomRight);
         sData.BoundingPlanes[4].SetFromThreePoints(sData.NearTopLeft,sData.NearTopRight,sData.NearBottomRight);
         sData.BoundingPlanes[5].SetFromThreePoints(sData.FarTopRight,sData.FarTopLeft,sData.FarBottomLeft);

         /* why is this a map? so far all cameras have been accessed by index?? */
         std::map<std::string, CCamerasSensorSimulatedAlgorithm::TMap, std::less<std::string> >::iterator
                itCamera = m_mapSimulatedAlgorithms.find(m_pcCameraEquippedEntity->GetCamera(un_cam_idx).GetId());

         if(itCamera != m_mapSimulatedAlgorithms.end()) {
            for(CCamerasSensorSimulatedAlgorithm::TMap::iterator itAlgorithm = itCamera->second.begin();
                itAlgorithm != itCamera->second.end();
                ++itAlgorithm) {
               /* Update the algorithm (transfers readings directly to the CI algorithm) */
               itAlgorithm->second->Update();
               /* transfer the checked rays to the controllable entity */
               m_pcControllableEntity->GetCheckedRays().insert(m_pcControllableEntity->GetCheckedRays().end(),
                                                               itAlgorithm->second->GetCheckedRays().begin(),
                                                               itAlgorithm->second->GetCheckedRays().end());
               
            }
         }
         else {
            THROW_ARGOSEXCEPTION("Could not find camera in algorithms vector");
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
