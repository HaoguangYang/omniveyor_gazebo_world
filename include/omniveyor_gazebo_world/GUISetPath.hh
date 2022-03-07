/*
 * This is the header file of: <GUISetPath.cc>
 * Last modified: 1/23/21, by Yubing Han
*/
#ifndef _GUI_SET_PATH_HH_
#define _GUI_SET_PATH_HH_

#include <gazebo/common/Plugin.hh>
#include <gazebo/gui/GuiPlugin.hh>

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <gazebo/transport/transport.hh>
# include <gazebo/gui/gui.hh>
#endif

namespace gazebo
{
    class GAZEBO_VISIBLE GUISetPath : public GUIPlugin
    {
      Q_OBJECT


      /// \brief Constructor
      /// \param[in] _parent Parent widget
      public: explicit GUISetPath();

      /// \brief Destructor
      public: virtual ~GUISetPath();


      public: 
        QVBoxLayout *mainLayout;
        QPushButton *button;
        QListWidget *tgtlst;
        QCheckBox *cycl;
        QPushButton *okButton;
        QPushButton *rmvButton;
        QPushButton *rmvallButton;
        QGroupBox *groupBox;
        QHBoxLayout *layoutGroup;
        QMessageBox *msgBox;

      /// \brief Callback trigged when the set path button is pressed.
      private slots: void OnButton();
      
      private slots: void okOn();

      private slots: void rmvOn();

      private slots: void rmvallOn();

      /// \brief Node used to establish communication with gzserver.
      private: transport::NodePtr node;
      
      /// \brief Subsciber of selected messages.
      private: transport::SubscriberPtr selSub;
     
      /// \brief callback that received selection message
      protected: void cb(ConstSelectionPtr &_msg);

    };
}
#endif
