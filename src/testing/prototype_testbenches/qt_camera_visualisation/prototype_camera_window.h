#ifndef PROTOTYPE_CAMERA_WINDOW_H
#define PROTOTYPE_CAMERA_WINDOW_H

class QVBoxLayout;
class QGraphicsScene;
class QGraphicsView;

#include <argos3/plugins/robots/prototype/control_interface/ci_prototype_cameras_sensor.h>

#include <QDialog>

using namespace argos;

class CPrototypeCameraWindow : public QDialog {

   Q_OBJECT

public:

   typedef std::vector<CPrototypeCameraWindow*> TList;

public:

   CPrototypeCameraWindow(QWidget * pc_parent,
                          CCI_PrototypeCamerasSensor* pc_camera,
                          UInt32 un_index);
   ~CPrototypeCameraWindow();

   void Update();

public slots:

   //void EntitySelected(size_t un_index);
   //void EntityDeselected(size_t un_index);

private:

   CCI_PrototypeCamerasSensor* m_pcCamera;
   UInt32 m_unCameraIndex;

   /* Qt UI Objects */
   QVBoxLayout* m_pcLayout;
   QGraphicsScene* m_pcScene;
   QGraphicsView* m_pcViewport;
   
   
};

#endif
