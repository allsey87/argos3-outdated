#include "camera_user_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/entity/composable_entity.h>

#include <argos3/plugins/robots/prototype/control_interface/ci_cameras_sensor.h>

/********************************************************************************/
/********************************************************************************/

CCameraUserFunctions::CCameraUserFunctions() : 
   m_cSimulator(CSimulator::GetInstance()),
   m_cSpace(m_cSimulator.GetSpace()),
   m_bConnected(false),
   m_pcSensor(NULL) {}

/********************************************************************************/
/********************************************************************************/

CCameraUserFunctions::~CCameraUserFunctions() {
   while(!m_tWindows.empty()) {
      delete m_tWindows.back();
      m_tWindows.pop_back();
   }
}

/********************************************************************************/
/********************************************************************************/

void CCameraUserFunctions::DrawInWorld() {
   if(!m_bConnected) {
      QObject::connect(&GetOpenGLWidget(),
                       SIGNAL(EntitySelected(size_t)),
                       this,
                       SLOT(EntitySelected(size_t)));
      QObject::connect(&GetOpenGLWidget(),
                       SIGNAL(EntityDeselected(size_t)),
                       this,
                       SLOT(EntityDeselected(size_t)));
      QObject::connect(&GetOpenGLWidget(),
                       SIGNAL(StepDone(int)),
                       this,
                       SLOT(UpdateCameras(int)));
      m_bConnected = true;
   }
}

/********************************************************************************/
/********************************************************************************/

void CCameraUserFunctions::EntitySelected(size_t un_index) {
   CEntity::TVector& vecEntities = m_cSpace.GetRootEntityVector();
   
   CComposableEntity* pcComposableEntity = dynamic_cast<CComposableEntity*>(vecEntities[un_index]);
   
   if(pcComposableEntity != NULL) {
      if(pcComposableEntity->HasComponent("controller")) {
         CControllableEntity& cController = pcComposableEntity->GetComponent<CControllableEntity>("controller");
         
         if(cController.GetController().HasSensor("cameras")) {
            m_pcSensor = cController.GetController().GetSensor<CCI_CamerasSensor>("cameras");
            for(UInt32 i = 0; i < m_pcSensor->GetDescriptors().size(); ++i) {
               m_tWindows.push_back(new CCameraWindow(&GetOpenGLWidget(),m_pcSensor, i));
            }
         }
      }
   }
}   

/********************************************************************************/
/********************************************************************************/

void CCameraUserFunctions::EntityDeselected(size_t un_index) {
   while(!m_tWindows.empty()) {
      delete m_tWindows.back();
      m_tWindows.pop_back();
   }
}

/********************************************************************************/
/********************************************************************************/

void CCameraUserFunctions::UpdateCameras(int n_step) {
   for(UInt32 i = 0; i < m_tWindows.size(); i++) {
      m_tWindows[i]->Update();
   }
}

/********************************************************************************/
/********************************************************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CCameraUserFunctions, "camera_user_functions");
