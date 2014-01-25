#ifndef PROTOTYPE_CAMERA_USER_FUNCTIONS_H
#define PROTOTYPE_CAMERA_USER_FUNCTIONS_H

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/testing/prototype_testbenches/qt_camera_visualisation/prototype_camera_window.h>

using namespace argos;

class CPrototypeCameraUserFunctions : public QObject,
                                      public CQTOpenGLUserFunctions {

   Q_OBJECT

public:

   CPrototypeCameraUserFunctions();
   virtual ~CPrototypeCameraUserFunctions();

   virtual void DrawInWorld();

public slots:

   void EntitySelected(size_t un_index);
   void EntityDeselected(size_t un_index);
   void UpdateCameras(int n_step);

private:

   CSimulator& m_cSimulator;
   CSpace& m_cSpace;

   bool m_bConnected;
   CCI_PrototypeForwardsCameraSensor* m_pcSensor;
   CPrototypeCameraWindow::TList m_tWindows;
   
};

#endif
