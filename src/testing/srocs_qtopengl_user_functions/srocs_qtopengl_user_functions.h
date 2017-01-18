#ifndef SROCS_QTOPENGL_USER_FUNCTIONS_H
#define SROCS_QTOPENGL_USER_FUNCTIONS_H

namespace argos {
   class CPrototypeEntity;
   class CQtRobotUserInterfaceWidget;
}

class QDockWidget;
class QTextEdit;
class QTabWidget;

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/core/utility/datatypes/datatypes.h>


namespace argos {
   class CSRoCSQTOpenGLUserFunctions : public QObject,
                                       public CQTOpenGLUserFunctions {

      Q_OBJECT

   public:

      CSRoCSQTOpenGLUserFunctions();
      virtual ~CSRoCSQTOpenGLUserFunctions();

      virtual void Init(TConfigurationNode& t_tree);

   public slots:

      void EntitySelected(size_t un_index);
      void StepDone(int n_step);

   private:

      CSimulator& m_cSimulator;
      CSpace& m_cSpace;

      QDockWidget* m_pcUserFunctionsDock;
      QTabWidget* m_pcTabWidget;
    

      struct SUserInterface {
         SInt32 TabIndex;
         CQtRobotUserInterfaceWidget* Widget;
      };

      std::list<std::pair<std::string, SUserInterface> > m_lstRobotUserInterfaces;


     
   };
}
#endif
