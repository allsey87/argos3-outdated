#ifndef CAMERA_WINDOW_H
#define CAMERA_WINDOW_H

class QVBoxLayout;
class QGraphicsScene;
class QGraphicsView;

#include <argos3/plugins/robots/prototype/control_interface/ci_cameras_sensor.h>
#include <argos3/plugins/robots/prototype/control_interface/ci_cameras_sensor_algorithms/ci_cameras_sensor_leddetector_algorithm.h>

#include <QDialog>

using namespace argos;

class CCameraWindow : public QDialog {

   Q_OBJECT

public:

   typedef std::vector<CCameraWindow*> TList;

public:

   CCameraWindow(QWidget * pc_parent,
                          CCI_CamerasSensor* pc_camera,
                          UInt32 un_index);
   ~CCameraWindow();

   void Update();

public slots:

   //void EntitySelected(size_t un_index);
   //void EntityDeselected(size_t un_index);

private:

   CCI_CamerasSensor* m_pcCamera;
   UInt32 m_unCameraIndex;

   /* Qt UI Objects */
   QVBoxLayout* m_pcLayout;
   QGraphicsScene* m_pcScene;
   QGraphicsView* m_pcViewport;
   
   std::string m_strCameraId;
   /* Algorithms */
   CCI_CamerasSensorLEDDetectorAlgorithm* m_pcLEDDetectorAlgorithm;
      
   
   
};

#endif
