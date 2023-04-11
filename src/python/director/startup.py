# This script is executed in the main console namespace so
# that all the variables defined here become console variables.


import os

import PythonQt
import director.applogic as app
import numpy as np
from PythonQt import QtGui
from director import actionhandlers
from director import affordancepanel
from director import affordancemanager
from director import camerabookmarks
from director import cameracontrol
from director import cameracontrolpanel
from director import cameraview
from director import contextviewbehaviors
from director import drcargs
from director import objectmodel as om
from director import screengrabberpanel
from director import segmentation
from director import segmentationpanel
from director import segmentationroutines
from director import viewcolors
from director import viewframes
from director import visualization as vis


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

# init segmentation routines
groundHeight = 0.0
viewFrame = segmentation.transformUtils.frameFromPositionAndRPY([1, 1, groundHeight + 1.5], [0, 0, -120])
segmentationroutines.SegmentationContext.initWithUser(groundHeight, viewFrame)

robotViewBehavior = contextviewbehaviors.ContextViewBehaviors(view)

# setup
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

imageWidget = cameraview.ImageWidget(
    cameraview.imageManager,
    [],
    view,
    visible=False,
)
imageViewHandler = ToggleImageViewHandler(imageWidget)
#

screengrabberpanel.init(view, imageWidget)
affordanceManager = affordancemanager.AffordanceObjectModelManager(
    view
)
affordancePanel = affordancepanel.init(
    view, affordanceManager
)


print("===== director setup complete, calling scripts for further setup =====")

for scriptArgs in drcargs.args().scripts:
    exec(compile(open(scriptArgs[0], "rb").read(), scriptArgs[0], "exec"))

#selector.finishSetup()
