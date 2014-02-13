/**
 * @file <argos3/plugins/robot/prototype/simulator/prototype_camera_sensor.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "prototype_cameras_sensor.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/positional_indices/positional_index.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/plugins/simulator/entities/led_entity.h>
#include <argos3/plugins/simulator/media/led_medium.h>

#include <argos3/plugins/robots/prototype/simulator/camera_equipped_entity.h>

namespace argos {

   /****************************************/
   /****************************************/

   class CLEDCheckOperation : public CPositionalIndex<CLEDEntity>::COperation {

   public:

      CLEDCheckOperation(bool b_show_rays,
                         CControllableEntity& c_controllable_entity) :
         m_bShowRays(b_show_rays),
         m_cControllableEntity(c_controllable_entity) {}

      virtual ~CLEDCheckOperation() {
         m_psReading->ObservationList.clear();
      }

      virtual bool operator()(CLEDEntity& c_led) {
         if(c_led.GetColor() != CColor::BLACK) {
            if((c_led.GetPosition() - m_sViewport.Position).Length() < m_sViewport.HalfExtents[0]) {
               m_cOcclusionCheckRay.SetEnd(c_led.GetPosition());         
               if(!GetClosestEmbodiedEntityIntersectedByRay(m_sIntersectionItem, m_cOcclusionCheckRay)) {
                  /* Take position of current LED */
                  CVector3 cLedPositionOnSensor = c_led.GetPosition();
                  /* Transform the position of LED into the local coordinate system of the camera */
                  cLedPositionOnSensor -= (m_cAttachedBodyPosition);
                  cLedPositionOnSensor.Rotate(m_cAttachedBodyOrientation.Inverse());
                  cLedPositionOnSensor -= m_cCameraPositionOffset;
                  cLedPositionOnSensor.Rotate(m_cCameraOrientationOffset.Inverse());
                  cLedPositionOnSensor.Rotate(CQuaternion(m_cCameraRoll, CVector3::Z));
                  /* Calculate the relevant index of the pixel presenting the centroid of the detected LED blob */
                  UInt32 unLedHorizontalIndex = m_unHorizontalResolution * (cLedPositionOnSensor.GetX() + m_sViewport.HalfExtents[0]) / (2.0f * m_sViewport.HalfExtents[0]);
                  UInt32 unLedVerticalIndex = m_unVerticalResolution * (cLedPositionOnSensor.GetY() + m_sViewport.HalfExtents[0]) / (2.0f * m_sViewport.HalfExtents[0]);
                  /* Store this observation into our observation list */
                  m_psReading->ObservationList.push_back(
                     CCI_PrototypeCamerasSensor::SObservation(c_led.GetColor(), unLedHorizontalIndex, unLedVerticalIndex));
                  if(m_bShowRays) {
                     m_cControllableEntity.AddCheckedRay(false, CRay3(m_cCameraPosition, c_led.GetPosition()));
                  }
               }
               else {
                  m_cControllableEntity.AddCheckedRay(true, CRay3(m_cCameraPosition, c_led.GetPosition()));
               }
            }
         }
         return true;
      }

      void Setup(CPrototypeCamerasSensor::SReading& s_reading,
                 const CPrototypeCamerasSensor::SViewport& s_viewport,
                 const CVector3& c_camera_position,
                 const CVector3& c_attached_body_position,
                 const CQuaternion& c_attached_body_orientation,
                 const CVector3& c_camera_position_offset,
                 const CQuaternion& c_camera_orientation_offset,
                 const CRadians& c_camera_roll,
                 UInt32 un_horizontal_resolution,
                 UInt32 un_vertical_resolution) {

         m_psReading = &s_reading;
         m_sViewport = s_viewport;
         m_cCameraPosition = c_camera_position;
         m_cAttachedBodyPosition = c_attached_body_position;
         m_cAttachedBodyOrientation = c_attached_body_orientation;
         m_cCameraPositionOffset = c_camera_position_offset;
         m_cCameraOrientationOffset = c_camera_orientation_offset;
         m_cCameraRoll = c_camera_roll;
         m_unHorizontalResolution = un_horizontal_resolution;
         m_unVerticalResolution = un_vertical_resolution;
         /* delete the previous observations */
         m_psReading->ObservationList.clear();         
         /* Set the beginning of the occulusion ray to the camera */
         m_cOcclusionCheckRay.SetStart(m_cCameraPosition);
      }
      
   private:
      bool m_bShowRays;
      CRay3 m_cOcclusionCheckRay;
      CControllableEntity& m_cControllableEntity;
      SEmbodiedEntityIntersectionItem m_sIntersectionItem;

      CPrototypeCamerasSensor::SReading* m_psReading;
      CPrototypeCamerasSensor::SViewport m_sViewport;

      CVector3    m_cCameraPosition;
      CVector3    m_cAttachedBodyPosition;
      CQuaternion m_cAttachedBodyOrientation;
      CVector3    m_cCameraPositionOffset;
      CQuaternion m_cCameraOrientationOffset;
      CRadians    m_cCameraRoll;

      UInt32 m_unHorizontalResolution;
      UInt32 m_unVerticalResolution;
   };

   /****************************************/
   /****************************************/

   CPrototypeCamerasSensor::CPrototypeCamerasSensor() :
      m_bEnabled(false),
      m_pcCamerasEntity(NULL),
      m_pcControllableEntity(NULL),
      m_pcLEDIndex(NULL),
      m_pcEmbodiedIndex(NULL),
      m_fDistanceNoiseStdDev(0.0f),
      m_pcRNG(NULL),
      m_bShowRays(false) {}

   /****************************************/
   /****************************************/

   CPrototypeCamerasSensor::~CPrototypeCamerasSensor() {
   }

   /****************************************/
   /****************************************/

   void CPrototypeCamerasSensor::SetRobot(CComposableEntity& c_entity) {
      /* Get and enable omndirectional camera equipped entity */
      m_pcCamerasEntity = &(c_entity.GetComponent<CCameraEquippedEntity>("camera_container"));
      m_pcCamerasEntity->SetCanBeEnabledIfDisabled(true);
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
      /* Get controllable entity */
      m_pcControllableEntity = &(c_entity.GetComponent<CControllableEntity>("controller"));
   }

   /****************************************/
   /****************************************/

   void CPrototypeCamerasSensor::Init(TConfigurationNode& t_tree) {
      try {
         /* Parent class init */
         CCI_PrototypeCamerasSensor::Init(t_tree);
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
         m_pcOperation = new CLEDCheckOperation(m_bShowRays, *m_pcControllableEntity);
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Error initializing the prototype camera sensor", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CPrototypeCamerasSensor::Update() {
      for(size_t i = 0; i < m_pcCamerasEntity->GetAllCameras().size(); ++i) {
         /* Calculate camera parameters */
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
         CVector3 cCameraPosition = m_pcCamerasEntity->GetOffsetPosition(i);
         cCameraPosition.Rotate(m_pcCamerasEntity->GetPositionalEntity(i).GetOrientation());
         cCameraPosition += m_pcCamerasEntity->GetPositionalEntity(i).GetPosition();
         /* Setup up the operation for camera i */
         m_pcOperation->Setup(m_tReadings[i],
                              m_vecViewports[i],
                              cCameraPosition,
                              m_pcCamerasEntity->GetPositionalEntity(i).GetPosition(),
                              m_pcCamerasEntity->GetPositionalEntity(i).GetOrientation(),
                              m_pcCamerasEntity->GetOffsetPosition(i),
                              m_pcCamerasEntity->GetOffsetOrientation(i),
                              m_pcCamerasEntity->GetCamera(i).GetRoll(),
                              m_pcCamerasEntity->GetCamera(i).GetHorizontalResolution(),
                              m_pcCamerasEntity->GetCamera(i).GetVerticalResolution());
         /* Run the LED check operation over all entities found in the index bounded or intersected by a box */
         m_pcLEDIndex->ForEntitiesInBoxRange(m_vecViewports[i].Position, m_vecViewports[i].HalfExtents, *m_pcOperation);
      }
   }

   /****************************************/
   /****************************************/

   void CPrototypeCamerasSensor::Reset() {
   }

   /****************************************/
   /****************************************/

   void CPrototypeCamerasSensor::Destroy() {
      delete m_pcOperation;
   }

   /****************************************/
   /****************************************/

   void CPrototypeCamerasSensor::Enable() {
   }

   /****************************************/
   /****************************************/

   void CPrototypeCamerasSensor::Disable() {
   }

   /****************************************/
   /****************************************/

   REGISTER_SENSOR(CPrototypeCamerasSensor,
                   "prototype_cameras", "default",
                   "Michael Allwright [allsey87@gmail.com]",
                   "1.0",
                   "A generic camera sensor to detect LEDs.",
                   "TODO\n\n",
                   "Usable"
		  );

}
