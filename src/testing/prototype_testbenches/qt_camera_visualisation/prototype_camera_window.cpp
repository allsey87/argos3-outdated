#include "prototype_camera_window.h"

/********************************************************************************/
/********************************************************************************/

CPrototypeCameraWindow::CPrototypeCameraWindow(QWidget* pc_parent,
                                               CCI_PrototypeForwardsCameraSensor* pc_camera,
                                               UInt32 un_index) :
   QDialog(pc_parent),
   m_pcCamera(pc_camera),
   m_unCameraIndex(un_index) {
   
   Update();

   /* window configuration */
   setWindowTitle(m_pcCamera->GetDescriptors()[m_unCameraIndex].Id.c_str());
   show();
}


/********************************************************************************/
/********************************************************************************/

CPrototypeCameraWindow::~CPrototypeCameraWindow() {
}

/********************************************************************************/
/********************************************************************************/

void CPrototypeCameraWindow::Update() {

}


/********************************************************************************/
/********************************************************************************/
