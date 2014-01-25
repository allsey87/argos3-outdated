#ifndef PROTOTYPE_CAMERA_WINDOW_H
#define PROTOTYPE_CAMERA_WINDOW_H

#include <argos3/plugins/robots/prototype/control_interface/ci_prototype_forwards_camera_sensor.h>

#include <QDialog>


using namespace argos;

class CPrototypeCameraWindow : public QDialog {

   Q_OBJECT

public:

   typedef std::vector<CPrototypeCameraWindow*> TList;

public:

   CPrototypeCameraWindow(QWidget * pc_parent,
                          CCI_PrototypeForwardsCameraSensor* pc_camera,
                          UInt32 un_index);
   ~CPrototypeCameraWindow();

   void Update();

public slots:

   //void EntitySelected(size_t un_index);
   //void EntityDeselected(size_t un_index);

private:

   CCI_PrototypeForwardsCameraSensor* m_pcCamera;
   UInt32 m_unCameraIndex;
   
};

#endif
