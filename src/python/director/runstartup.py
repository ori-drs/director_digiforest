import director
import sys
import os
import signal
from collections import OrderedDict

import director.applogic as app
from director.digiforest import forestpayloadspanel

from director import applogic
from director import cameraview

from director import tasklaunchpanel
from director import viewcolors
from director import screengrabberpanel


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


def startup(globalsDict=None):
    # Panels:
    nodeImageManager = cameraview.StaticImageManager()
    view = app.getDRCView()
    imageWidget = cameraview.StaticImageWidget(
        imageManager=nodeImageManager,
        imageNames=[],
        view=view,
        visible=False
    )
    imageViewHandler = ToggleImageViewHandler(imageWidget)

    forestPanel = forestpayloadspanel.init(nodeImageManager)

    globalsDict["forestPanel"] = forestPanel

startup(globals())
