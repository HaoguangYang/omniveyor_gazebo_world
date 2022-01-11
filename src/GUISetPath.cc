/* This program customized the Gazebo GUI to allow users to click a bunch of targets in order to set the path of the moving base. 
 * Header file: <GUISetPath.hh>
 * Last modified: 1/23/21, by Yubing Han
*/

#include <iostream>
#include <sstream>
#include <gazebo/msgs/msgs.hh>
#include "GUISetPath.hh"
#include "string"
#include "std_msgs/String.h"
#include "list"

using namespace gazebo;

std::string modName;
std::string frmMsg;
std::string toMsg;
std::list<std::string> tgt;
QString temp;
int setClicked = -1;

// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(GUISetPath)

/////////////////////////////////////////////////
GUISetPath::GUISetPath()
  : GUIPlugin()
{ 
  // Set the frame background and foreground colors
  this->setStyleSheet(
      "QFrame { background-color : rgba(100, 100, 100, 255); color : white; }");

  // Create the main layout
  mainLayout = new QVBoxLayout;

  // Create a push button, and connect it to the OnButton function
  button = new QPushButton("Set path",this);
  connect(button, SIGNAL(clicked()), this, SLOT(OnButton()));
  
  //Create target list
  tgtlst = new QListWidget();

  
  //Create cyclic option
  cycl = new QCheckBox("is cyclic?", this);

  // Create ok and cancel buttons
  groupBox = new QGroupBox();
  okButton = new QPushButton("Submit",this);
  connect(okButton, SIGNAL(clicked()), this, SLOT(okOn()));
  okButton->setEnabled(false);
  rmvButton = new QPushButton("Remove",this);
  connect(rmvButton, SIGNAL(clicked()), this, SLOT(rmvOn())); 
  rmvallButton = new QPushButton("Remove all",this);
  connect(rmvallButton, SIGNAL(clicked()), this, SLOT(rmvallOn()));
  layoutGroup = new QHBoxLayout;
  layoutGroup->addWidget(okButton);
  layoutGroup->addWidget(rmvButton);
  layoutGroup->addWidget(rmvallButton);
  layoutGroup->setContentsMargins(0, 0, 10, 0);
  groupBox->setLayout(layoutGroup);

  //Add widgets
  mainLayout->addWidget(button);
  mainLayout->addWidget(tgtlst);
  mainLayout->addWidget(cycl);
  mainLayout->addWidget(groupBox);

  // Remove margins to reduce space
  mainLayout->setContentsMargins(0, 0, 0, 0);

  this->setLayout(mainLayout);

  // Position and resize this widget
  this->move(10, 10);
  this->resize(300, 280);

  //Create node
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  this->selSub = this->node->Subscribe("~/selection", &GUISetPath::cb, this);
}

/////////////////////////////////////////////////
GUISetPath::~GUISetPath()
{
}

/////////////////////////////////////////////////
void GUISetPath::OnButton()
{
  setClicked = 0;  
}


// Function is called everytime a message is received.
void GUISetPath::cb(ConstSelectionPtr &_msg)
{

  std::stringstream ss(_msg->DebugString());
  std::istream_iterator<std::string> begin(ss);
  std::istream_iterator<std::string> end;
  std::vector<std::string> vstrings(begin, end);
  if (vstrings[5] == "true"){
    modName = vstrings[3];
    std::cout<<modName<<" selected......\n";
    if (setClicked >= 0){
      temp = QString::fromStdString(modName);
      this->tgtlst->addItem(temp);
      this->okButton->setEnabled(true);
      setClicked++;
    }
  }
}

/////////////////////////////////////////////////
void GUISetPath::okOn()
{
  msgBox = new QMessageBox;
  msgBox->setWindowTitle("IMI");
  msgBox->setText("Request sent");
  msgBox->exec();
  setClicked = -1;
  this->tgtlst->clear();
  this->cycl->setChecked(false);
  this->okButton->setEnabled(false);
}

/////////////////////////////////////////////////
void GUISetPath::rmvOn()
{
  this->tgtlst->takeItem(setClicked-1);
  setClicked--;
}

/////////////////////////////////////////////////
void GUISetPath::rmvallOn()
{ 
  this->tgtlst->clear();  
  setClicked = -1;
  this->cycl->setChecked(false);
  this->okButton->setEnabled(false);
}
