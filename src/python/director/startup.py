# This script is executed in the main console namespace so
# that all the variables defined here become console variables.


import os

import PythonQt
import director.applogic as app
import numpy as np
from PythonQt import QtGui
from director import actionhandlers
from director import affordancepanel
from director import camerabookmarks
from director import cameracontrol
from director import cameracontrolpanel
from director import cameraview
from director import drcargs
from director import framevisualization
from director import objectmodel as om
from director import robotsystem
from director import screengrabberpanel
from director import segmentation
from director import segmentationpanel
from director import viewcolors
from director import viewframes
from director import visualization as vis
from director import vtkAll as vtk
from director.debugpolydata import DebugData
from director.pointpicker import ImagePointPicker
from director.timercallback import TimerCallback


class ToggleImageViewHandler(object):
    def __init__(self, manager):
        self.action = app.getToolsMenuActions()["ActionToggleImageView"]
        self.action.connect("triggered()", self.toggle)
        self.manager = manager

    def toggle(self):
        if self.action.checked:
            self.manager.show()
        else:
            self.manager.hide()


class RobotGridUpdater(object):
    def __init__(self, gridFrame, robotModel, jointController, z_offset=0):
        self.gridFrame = gridFrame
        self.robotModel = robotModel
        self.jointController = jointController
        self.robotModel.connectModelChanged(self.updateGrid)
        print("Setting z offset to {}".format(z_offset))
        self.z_offset = z_offset

    def setZOffset(self, z_offset):
        self.z_offset = z_offset
        self.updateGrid(None)

    def updateGrid(self, model):
        pos = self.jointController.q[:3]

        x = int(np.round(pos[0])) / 10
        y = int(np.round(pos[1])) / 10
        z = np.round((pos[2] - self.z_offset) * 10.0) / 10.0

        t = vtk.vtkTransform()
        t.Translate((x * 10, y * 10, z))
        self.gridFrame.copyFrame(t)


drcargs.args()
app.startup(globals())
om.init(app.getMainWindow().objectTree(), app.getMainWindow().propertiesPanel())
actionhandlers.init()

quit = app.quit
exit = quit
view = app.getDRCView()
camera = view.camera()
tree = app.getMainWindow().objectTree()
orbit = cameracontrol.OrbitController(view)
showPolyData = segmentation.showPolyData
updatePolyData = segmentation.updatePolyData


# selector = RobotSelector()
# # To hide the selector if there is only one robot we actually need to hide the action that is created by the
# # toolbar's addwidget
# selectorAction = app.getMainWindow().toolBar().addWidget(selector)

# If this is a single robot configuration, we expect modelName as a top level key. Otherwise it will be a second
# level one.
robotSystems = []
for (
    _,
    robotConfig,
) in drcargs.DirectorConfig.getDefaultInstance().robotConfigs.items():
    print("Loading config for robot with name {}".format(robotConfig["robotName"]))
    robotSystems.append(robotsystem.create(view, robotName=robotConfig["robotName"]))

# If there is only one robot, the selector should not be shown
# if len(robotSystems) == 1:
#     selectorAction.setVisible(False)
#     # When there is only one robot we do not want to prefix topics
#     robotSystems[0]._add_fields(rosPrefix="", single=True)
# else:
#     for robotSystem in robotSystems:
#         # With multiple robots, prefix the topics with the robot names
#         robotSystem._add_fields(rosPrefix=robotSystem.robotName, single=False)

# Before going through all the robot systems, we do some setup which is universal and not linked to any specific robot
useLightColorScheme = True

sceneRoot = om.getOrCreateContainer("scene")
grid = vis.showGrid(view, color=[0, 0, 0], alpha=0.1, parent=sceneRoot)
grid.setProperty("Surface Mode", "Surface with edges")

app.setBackgroundColor([0.3, 0.3, 0.35], [0.95, 0.95, 1])

viewOptions = vis.ViewOptionsItem(view)
om.addToObjectModel(viewOptions, parentObj=sceneRoot)

viewBackgroundLightHandler = viewcolors.ViewBackgroundLightHandler(
    viewOptions, grid, app.getToolsMenuActions()["ActionToggleBackgroundLight"]
)

viewFramesHandler = viewframes.ViewFramesSizeHandler(
    app.getToolsMenuActions()["ActionToggleFramesSize"]
)

if not useLightColorScheme:
    viewBackgroundLightHandler.action.trigger()

cameraBooksmarksPanel = camerabookmarks.init(view)

cameraControlPanel = cameracontrolpanel.CameraControlPanel(view)
app.addWidgetToDock(cameraControlPanel.widget, action=None).hide()

app.setCameraTerrainModeEnabled(view, True)
app.resetCamera(viewDirection=[-1, 0, 0], view=view)

for robotSystem in robotSystems:
    # selector.addRobot(robotSystem.robotName)
    # selector.associateViewBehaviorWithRobot(
    #     robotSystem.viewBehaviors, robotSystem.robotName
    # )
    directorConfig = drcargs.getRobotConfig(robotSystem.robotName)
    #
    # usePerception = True
    # useDataFiles = True
    #
    # useSkybox = False
    # useFeetlessRobot = False
    #
    # poseCollection = PythonQt.dd.ddSignalMap()
    # costCollection = PythonQt.dd.ddSignalMap()
    #
    # if "disableComponents" in directorConfig:
    #     for component in directorConfig["disableComponents"]:
    #         print("Disabling", component)
    #         locals()[component] = False
    #
    # if "enableComponents" in directorConfig:
    #     for component in directorConfig["enableComponents"]:
    #         print("Enabling", component)
    #         locals()[component] = True
    #
    # if usePerception:
    #     segmentationpanel.init()
    #     cameraview.init(robotName=robotSystem.robotName)
    #
    #     cameraview.cameraViews[
    #         robotSystem.robotName
    #     ].rayCallback = segmentation.extractPointsAlongClickRay
    #
    # gridUpdater = RobotGridUpdater(
    #     grid.getChildFrame(),
    #     robotSystem.robotStateModel,
    #     robotSystem.robotStateJointController,
    #     directorConfig.get("grid_z_offset", 0),
    # )
    #
    #
    # #reset time button and connections
    # reset_time_button = QtGui.QPushButton("Reset time")
    # reset_time_button.setObjectName("resettime")
    #
    # # Iterate over all sources and reset time when reset button pressed
    # def reset_sources_time():
    #     for source in robotSystem.sources:
    #         if hasattr(source, "resetTime"):
    #             source.resetTime()
    #         else:
    #             print(
    #                 "WARNING: source {} does not have a resetTime function. This is probably a mistake.".format(
    #                     source
    #                 )
    #             )
    #
    # reset_time_button.connect("clicked()", reset_sources_time)
    # reset_time_button.connect(
    #     "clicked()", cameraview.cameraViews[robotSystem.robotName].resetTime
    # )
    # app.getMainWindow().statusBar().addPermanentWidget(reset_time_button)
    # app.getRobotSelector().associateWidgetWithRobot(
    #     reset_time_button, robotSystem.robotName
    # )
    #
    # if useSkybox:
    #     skyboxDataDir = os.path.expanduser("~/Downloads/skybox")
    #     imageMap = skybox.getSkyboxImages(skyboxDataDir)
    #     skyboxObjs = skybox.createSkybox(imageMap, view)
    #     skybox.connectSkyboxCamera(view)
    #
    # robotHighlighter = RobotLinkHighlighter(robotSystem.robotStateModel)
    #
    # if useDataFiles:
    #
    #     for filename in drcargs.args().data_files:
    #         actionhandlers.onOpenFile(filename)
    #
    cameras = [
        camera["name"] for camera in directorConfig["sensors"]["camera"]["color"]
    ]
    #imageOverlayManager = ImageOverlayManager(cameras, robotSystem.robotName)
    imageWidget = cameraview.ImageWidget(
        cameraview.imageManager,
        cameras,
        view,
        visible=False,
        robotName=robotSystem.robotName,
    )
    imageViewHandler = ToggleImageViewHandler(imageWidget)
    #

    screengrabberpanel.init(view, imageWidget, robotSystem.robotName)
    framevisualization.init(view, robotSystem)
    affordancePanel = affordancepanel.init(
        view, robotSystem.affordanceManager, robotSystem.robotStateJointController
    )

    # def drawCenterOfMass(model):
    #     stanceFrame = robotSystem.footstepsDriver.getFeetMidPoint(model)
    #     com = list(model.model.getCenterOfMass())
    #     com[2] = stanceFrame.GetPosition()[2]
    #     d = DebugData()
    #     d.addSphere(com, radius=0.015)
    #     obj = vis.updatePolyData(
    #         d.getPolyData(),
    #         "COM %s" % model.getProperty("Name"),
    #         color=[1, 0, 0],
    #         visible=False,
    #         parent=model,
    #     )
    #
    # def initCenterOfMassVisualization():
    #     for model in [
    #         robotSystem.robotStateModel,
    #         robotSystem.teleopRobotModel,
    #         robotSystem.robotSystem.playbackRobotModel,
    #     ]:
    #         model.connectModelChanged(drawCenterOfMass)
    #         drawCenterOfMass(model)
    #
    # if useFeetlessRobot:
    #     robotSystem.ikPlanner.robotNoFeet = True

print("===== director setup complete, calling scripts for further setup =====")

for scriptArgs in drcargs.args().scripts:
    exec(compile(open(scriptArgs[0], "rb").read(), scriptArgs[0], "exec"))

#selector.finishSetup()
