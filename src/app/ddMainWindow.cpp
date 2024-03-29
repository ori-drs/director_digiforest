#include "ddMainWindow.h"

#include "ddMacros.h"
#include "ddMacrosManager.h"
#include "ddPythonManager.h"
#include "ddViewManager.h"
#include "ddPropertiesPanel.h"
#include "ddViewMenu.h"

#include "ui_ddMainWindow.h"

#include <QApplication>
#include <QTimer>
#include <QShortcut>
#include <QStatusBar>
#include <QLabel>
#include <QPointer>

#include <cstdio>


//-----------------------------------------------------------------------------
class ddMainWindow::ddInternal : public Ui::ddMainWindow
{
public:

  ddViewManager* ViewManager;
  ddPropertiesPanel* PropertiesPanel;
  ddMacrosManager* ViewMacrosManager;
  ddMacrosManager* UserMacrosManager;
  QPointer<ddPythonManager> PythonManager;

  ddViewMenu* ViewMenuManager;
};


//-----------------------------------------------------------------------------
ddMainWindow::ddMainWindow()
{
  this->Internal = new ddInternal;
  this->Internal->setupUi(this);

  this->Internal->ViewManager = new ddViewManager;
  this->Internal->PropertiesPanel = new ddPropertiesPanel;

  this->setCentralWidget(this->Internal->ViewManager);
  this->Internal->PropertiesScrollArea->setWidget(this->Internal->PropertiesPanel);

  this->setWindowTitle("Director");
  this->connect(this->Internal->ActionQuit, SIGNAL(triggered()), QApplication::instance(), SLOT(quit()));

  this->Internal->OutputConsoleDock->hide();
  this->connect(QCoreApplication::instance(), SIGNAL(aboutToQuit()), this, SIGNAL(shuttingDown()));
  this->connect(this->Internal->ActionResetCamera, SIGNAL(triggered()), this, SIGNAL(resetCamera()));
  this->connect(this->Internal->ActionColorCloud, SIGNAL(triggered()), this, SIGNAL(colorCloud()));
  this->connect(this->Internal->ActionToggleStereoRender, SIGNAL(triggered()), this, SIGNAL(toggleStereoRender()));
  this->connect(this->Internal->ActionToggleCameraTerrainMode, SIGNAL(triggered()), this, SIGNAL(toggleCameraTerrainMode()));
  this->connect(this->Internal->ActionOpen, SIGNAL(triggered()), this, SIGNAL(fileOpen()));
  this->connect(this->Internal->ActionSaveData, SIGNAL(triggered()), this, SIGNAL(fileSaveData()));
  this->connect(this->Internal->ActionOnlineHelp, SIGNAL(triggered()), this, SIGNAL(openOnlineHelp()));

  QTimer::singleShot(0, this, SLOT(startup()));

  this->setCorner(Qt::TopLeftCorner, Qt::LeftDockWidgetArea);
  this->setCorner(Qt::TopRightCorner, Qt::RightDockWidgetArea);
  this->setCorner(Qt::BottomLeftCorner, Qt::LeftDockWidgetArea);
  this->setCorner(Qt::BottomRightCorner, Qt::RightDockWidgetArea);

  // This disables right-click context menus for the main window, which show all the dock widgets
  // Doing this means we don't have to handle enabling/disabling each individual item in the context menu
  this->setContextMenuPolicy(Qt::NoContextMenu);

  //QLabel* logoLabel = new QLabel();
  //logoLabel->setPixmap(QPixmap(":/images/drake_logo.png").scaled(QSize(32,32),  Qt::KeepAspectRatio));
  //logoLabel->setScaledContents(true);
  //this->statusBar()->addPermanentWidget(logoLabel);

  this->setupViewMenu();

  this->Internal->ViewMacrosManager = new ddMacrosManager(this);
  this->Internal->ViewMacrosManager->setToolBar(this->Internal->ViewToolBar);

  this->Internal->UserMacrosManager = new ddMacrosManager(this);
  this->Internal->UserMacrosManager->setToolBar(this->Internal->MacrosToolBar);

  this->connect(this->Internal->ViewManager, SIGNAL(currentViewChanged(ddViewBase*, ddViewBase*)), SLOT(onCurrentViewChanged(ddViewBase*, ddViewBase*)));
  this->onCurrentViewChanged(0, this->Internal->ViewManager->currentView());
}

//-----------------------------------------------------------------------------
ddMainWindow::~ddMainWindow()
{
  delete this->Internal;
}

//-----------------------------------------------------------------------------
ddViewManager* ddMainWindow::viewManager() const
{
  return this->Internal->ViewManager;
}

//-----------------------------------------------------------------------------
ddPropertiesPanel* ddMainWindow::propertiesPanel() const
{
  return this->Internal->PropertiesPanel;
}

//-----------------------------------------------------------------------------
QToolBar* ddMainWindow::toolBar() const
{
  return this->Internal->MainToolBar;
}

//-----------------------------------------------------------------------------
QToolBar* ddMainWindow::macrosToolBar() const
{
  return this->Internal->MacrosToolBar;
}

//-----------------------------------------------------------------------------
QToolBar* ddMainWindow::panelToolBar() const
{
  return this->Internal->PanelToolBar;
}

//-----------------------------------------------------------------------------
ddObjectTree* ddMainWindow::objectTree() const
{
  return this->Internal->ObjectTree;
}

//-----------------------------------------------------------------------------
QTextEdit* ddMainWindow::outputConsole() const
{
  return this->Internal->OutputConsole;
}

//-----------------------------------------------------------------------------
QMenu* ddMainWindow::toolsMenu() const
{
  return this->Internal->ToolsMenu;
}

//-----------------------------------------------------------------------------
void ddMainWindow::toggleOutputConsoleVisibility()
{
  if (this->Internal->OutputConsoleDock->isHidden())
  {
    this->Internal->OutputConsoleDock->show();
  }
  else
  {
    this->Internal->OutputConsoleDock->hide();
  }
}

//-----------------------------------------------------------------------------
void ddMainWindow::addWidgetToViewMenu(QWidget* widget, const QString& subMenuName)
{
  if (!widget)
  {
    return;
  }

  this->Internal->ViewMenuManager->addWidget(widget, widget->windowTitle(), QIcon(), subMenuName);
}

//-----------------------------------------------------------------------------
void ddMainWindow::addWidgetToViewMenu(QWidget* widget, QAction* action, const QString& subMenuName)
{
  this->Internal->ViewMenuManager->addWidget(widget, action, subMenuName);
}

//-----------------------------------------------------------------------------
QList<QAction*> ddMainWindow::toolBarActions() const
{
  QList<QToolBar*> toolBars;
  toolBars << this->Internal->MainToolBar << this->Internal->PanelToolBar;

  QList<QAction*> actions;
  foreach (QToolBar* toolBar, toolBars)
  {
    actions << toolBar->actions();
  }

  return actions;
}

//-----------------------------------------------------------------------------
void ddMainWindow::onCurrentViewChanged(ddViewBase* previousView, ddViewBase* currentView)
{

}

//-----------------------------------------------------------------------------
void ddMainWindow::closeEvent(QCloseEvent *event)
{
  QMainWindow::closeEvent(event);
  QCoreApplication::instance()->quit();
}

//-----------------------------------------------------------------------------
void ddMainWindow::handleCommandLineArgs()
{
  QStringList args = QApplication::instance()->arguments();
  ddNotUsed(args);
}

//-----------------------------------------------------------------------------
void ddMainWindow::startup()
{
  this->handleCommandLineArgs();

  this->setupPython();
  QString startupScript = ddPythonManager::appSitePackagesDir() + "/director/startup.py";
  this->Internal->PythonManager->executeFile(startupScript);
}

//----------------------------------------------------------------------------
void ddMainWindow::setupViewMenu()
{
  ddViewMenu* viewMenu = new ddViewMenu(*this->Internal->ViewMenu, this);
  this->Internal->ViewMenuManager = viewMenu;
  ddViewMenu* toolbarMenu = new ddViewMenu(*this->Internal->ToolBarMenu, this);

  viewMenu->addWidget(
    this->Internal->ObjectsDock,
    this->Internal->ObjectsDock->windowTitle());

  viewMenu->addWidget(
    this->Internal->PropertiesDock,
    this->Internal->PropertiesDock->windowTitle());

  viewMenu->addWidget(
    this->Internal->OutputConsoleDock,
    this->Internal->OutputConsoleDock->windowTitle());

  toolbarMenu->addWidget(
    this->Internal->MainToolBar,
    this->Internal->MainToolBar->windowTitle());

  toolbarMenu->addWidget(
    this->Internal->ViewToolBar,
    this->Internal->ViewToolBar->windowTitle());

  toolbarMenu->addWidget(
    this->Internal->MacrosToolBar,
    this->Internal->MacrosToolBar->windowTitle());
}

//-----------------------------------------------------------------------------
void ddMainWindow::setPythonManager(ddPythonManager* pythonManager)
{
  this->Internal->PythonManager = pythonManager;
}

//-----------------------------------------------------------------------------
void ddMainWindow::setupPython()
{
  this->Internal->PythonManager->setSysArgv(QApplication::instance()->arguments());
  this->Internal->PythonManager->addObjectToPythonMain("_mainWindow", this);
  this->Internal->PythonManager->setupConsole(this);
  this->connect(this->Internal->ActionPythonConsole, SIGNAL(triggered()), this->Internal->PythonManager, SLOT(showConsole()));

  this->Internal->PythonManager->connect(this->Internal->ViewMacrosManager, SIGNAL(executeScriptRequested(const QString&)), SLOT(onExecuteFile(const QString&)));
  this->Internal->PythonManager->connect(this->Internal->UserMacrosManager, SIGNAL(executeScriptRequested(const QString&)), SLOT(onExecuteFile(const QString&)));
}
