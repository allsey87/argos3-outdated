#ifndef QT_ROBOT_USER_INTERFACE_WIDGET_H
#define QT_ROBOT_USER_INTERFACE_WIDGET_H

namespace argos {
   class CPrototypeEntity;
   class CCI_Controller;
   class CCI_CamerasSensorTagDetectorAlgorithm;
   class CCI_CamerasSensorLEDDetectorAlgorithm;
   class CCI_CamerasSensor;
   class CQTOpenGLLogStream;
}

class QVBoxLayout;
class QGraphicsScene;
class QGraphicsView;
class QTextEdit;

#include <QWidget>

namespace argos {
   class CQtRobotUserInterfaceWidget : public QWidget {
      Q_OBJECT

   public:
      CQtRobotUserInterfaceWidget(QWidget* pc_parent,
                                  CCI_Controller* pc_controller);
      ~CQtRobotUserInterfaceWidget();

      void Update();

   private:
      /* Qt UI Objects */
      QVBoxLayout* m_pcLayout;
      QGraphicsScene* m_pcScene;
      QGraphicsView* m_pcViewport;
      QTextEdit* m_pcLogTargetsBuffer;
      QTextEdit* m_pcLogStatesBuffer;

      CQTOpenGLLogStream* m_pcLogTargetsStream;
      CQTOpenGLLogStream* m_pcLogStatesStream;

      /* Reference to the robot */
      CCI_Controller* m_pcController;

      CCI_CamerasSensor* m_pcCamera;

      /* Algorithms */
      CCI_CamerasSensorLEDDetectorAlgorithm* m_pcLEDDetectorAlgorithm;
      CCI_CamerasSensorTagDetectorAlgorithm* m_pcTagDetectorAlgorithm;
      
   };
}
#endif
