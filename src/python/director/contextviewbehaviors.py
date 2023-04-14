import PythonQt
from PythonQt import QtCore, QtGui
import director.objectmodel as om
import director.visualization as vis
from director import affordanceitems
from director import callbacks
from director import cameracontrol
from director import transformUtils
from director.debugpolydata import DebugData
from director.pointpicker import PlacerWidget
from director import vtkNumpy as vnp
from director import applogic as app
from director import vtkAll as vtk
from director import filterUtils
from director.shallowCopy import shallowCopy
from director import segmentation
from director import segmentationroutines
from director.vieweventfilter import ViewEventFilter
from director import viewbehaviors
from director.utime import getUtime
from director import drcargs

import numpy as np
from . import ioutils
import os
import re
import random
import colorsys

lastRandomColor = 0.0


class ContextViewBehaviors(object):
    def __init__(self, view):
        self.viewBehaviors = viewbehaviors.ViewBehaviors(view)
        self.robotViewBehaviors = RobotViewEventFilter(view)


def getChildFrame(obj):
    if hasattr(obj, "getChildFrame"):
        return obj.getChildFrame()


def toggleFootstepWidget(displayPoint, view, useHorizontalWidget=False):

    obj, _ = vis.findPickedObject(displayPoint, view)

    if not obj:
        return False

    name = obj.getProperty("Name")

    if name in ("footstep widget", "footstep widget frame"):
        om.removeFromObjectModel(om.findObjectByName("footstep widget"))
        return True

    match = re.match("^step (\d+)$", name)
    if not match:
        return False

    stepIndex = int(match.group(1))

    existingWidget = om.findObjectByName("footstep widget")
    if existingWidget:
        previousStep = existingWidget.stepIndex
        om.removeFromObjectModel(existingWidget)
        if previousStep == stepIndex:
            return True

    footMesh = shallowCopy(obj.polyData)
    footFrame = transformUtils.copyFrame(obj.getChildFrame().transform)

    if useHorizontalWidget:
        rpy = [0.0, 0.0, transformUtils.rollPitchYawFromTransform(footFrame)[2]]
        footFrame = transformUtils.frameFromPositionAndRPY(
            footFrame.GetPosition(), np.degrees(rpy)
        )

    footObj = vis.showPolyData(
        footMesh, "footstep widget", parent="planning", alpha=0.2
    )
    footObj.stepIndex = stepIndex
    frameObj = vis.showFrame(
        footFrame, "footstep widget frame", parent=footObj, scale=0.2
    )
    footObj.actor.SetUserTransform(frameObj.transform)
    footObj.setProperty("Color", obj.getProperty("Color"))
    frameObj.setProperty("Edit", True)

    rep = frameObj.widget.GetRepresentation()
    rep.SetTranslateAxisEnabled(2, False)
    rep.SetRotateAxisEnabled(0, False)
    rep.SetRotateAxisEnabled(1, False)
    frameObj.widget.HandleRotationEnabledOff()

    walkGoal = om.findObjectByName("walking goal")
    if walkGoal:
        walkGoal.setProperty("Edit", False)

    return True


def getAsFrame(obj):
    if isinstance(obj, vis.FrameItem):
        return obj
    elif hasattr(obj, "getChildFrame"):
        return obj.getChildFrame()


def isGraspSeed(obj):
    return hasattr(obj, "side")


def getCollisionParent(obj):
    """
    If obj is an affordance, return obj
    If obj is a frame or a grasp seed, return first parent.
    """
    if isinstance(obj, vis.FrameItem):
        return obj.parent()
    if isGraspSeed(obj):
        return obj.parent()
    else:
        return obj


# The most recently cached PickedPoint - available as input to any other algorithm
lastCachedPickedPoint = np.array([0, 0, 0])


def getObjectAsPointCloud(obj):
    try:
        obj = obj.model.polyDataObj
    except AttributeError:
        pass

    try:
        obj.polyData
    except AttributeError:
        return None

    if (
        obj and obj.polyData.GetNumberOfPoints()
    ):  # and (obj.polyData.GetNumberOfCells() == obj.polyData.GetNumberOfVerts()):
        return obj


def getRobotActions(view, pickedObj, pickedPoint):

    reachFrame = getAsFrame(pickedObj)
    collisionParent = getCollisionParent(pickedObj)
    pointCloudObj = getObjectAsPointCloud(pickedObj)


    def flipHandSide():
        for obj in [pickedObj] + pickedObj.children():
            if not isGraspSeed(obj):
                continue
            side = "right" if obj.side == "left" else "left"
            obj.side = side
            color = [1.0, 1.0, 0.0]
            if side == "right":
                color = [0.33, 1.0, 0.0]
            obj.setProperty("Color", color)

            polyData = handFactory.getNewHandPolyData(side)
            obj.setPolyData(polyData)

            handFrame = obj.children()[0]
            t = transformUtils.copyFrame(handFrame.transform)
            t.PreMultiply()
            t.RotateY(180)
            handFrame.copyFrame(t)

            objName = obj.getProperty("Name")
            frameName = handFrame.getProperty("Name")
            if side == "left":
                obj.setProperty("Name", objName.replace("right", "left"))
                handFrame.setProperty("Name", frameName.replace("right", "left"))
            else:
                obj.setProperty("Name", objName.replace("left", "right"))
                handFrame.setProperty("Name", frameName.replace("left", "right"))
            obj._renderAllViews()

    def flipHandThumb():
        handFrame = pickedObj.children()[0]
        t = transformUtils.copyFrame(handFrame.transform)
        t.PreMultiply()
        t.RotateY(180)
        handFrame.copyFrame(t)
        pickedObj._renderAllViews()

    def onSegmentGround():
        groundPoints, scenePoints = segmentation.removeGround(pointCloudObj.polyData)
        vis.showPolyData(
            groundPoints, "ground points", color=[0, 1, 0], parent="segmentation"
        )
        vis.showPolyData(
            scenePoints, "scene points", color=[1, 0, 1], parent="segmentation"
        )
        pickedObj.setProperty("Visible", False)

    def onCopyPointCloud():
        global lastRandomColor
        polyData = vtk.vtkPolyData()
        polyData.DeepCopy(pointCloudObj.polyData)

        if pointCloudObj.getChildFrame():
            polyData = segmentation.transformPolyData(
                polyData, pointCloudObj.getChildFrame().transform
            )
        polyData = segmentation.addCoordArraysToPolyData(polyData)

        # generate random color, and average with a common color to make them generally similar
        lastRandomColor = lastRandomColor + 0.1 + 0.1 * random.random()
        rgb = colorsys.hls_to_rgb(lastRandomColor, 0.7, 1.0)
        obj = vis.showPolyData(
            polyData,
            pointCloudObj.getProperty("Name") + " copy",
            color=rgb,
            parent="point clouds",
        )

        # t = vtk.vtkTransform()
        # t.PostMultiply()
        # t.Translate(filterUtils.computeCentroid(polyData))
        # segmentation.makeMovable(obj, t)
        om.setActiveObject(obj)
        pickedObj.setProperty("Visible", False)

    def onMergeIntoPointCloud():
        allPointClouds = om.findObjectByName("point clouds")
        if allPointClouds:
            allPointClouds = [i.getProperty("Name") for i in allPointClouds.children()]
        sel = QtGui.QInputDialog.getItem(
            None,
            "Point Cloud Merging",
            "Pick point cloud to merge into:",
            allPointClouds,
            current=0,
            editable=False,
        )
        sel = om.findObjectByName(sel)

        # Make a copy of each in same frame
        polyDataInto = vtk.vtkPolyData()
        polyDataInto.ShallowCopy(sel.polyData)
        if sel.getChildFrame():
            polyDataInto = segmentation.transformPolyData(
                polyDataInto, sel.getChildFrame().transform
            )

        polyDataFrom = vtk.vtkPolyData()
        polyDataFrom.DeepCopy(pointCloudObj.polyData)
        if pointCloudObj.getChildFrame():
            polyDataFrom = segmentation.transformPolyData(
                polyDataFrom, pointCloudObj.getChildFrame().transform
            )

        # Actual merge
        append = filterUtils.appendPolyData([polyDataFrom, polyDataInto])
        if sel.getChildFrame():
            polyDataInto = segmentation.transformPolyData(
                polyDataInto, sel.getChildFrame().transform.GetInverse()
            )

        # resample
        append = segmentationroutines.applyVoxelGrid(append, 0.01)
        append = segmentation.addCoordArraysToPolyData(append)

        # Recenter the frame
        sel.setPolyData(append)
        t = vtk.vtkTransform()
        t.PostMultiply()
        t.Translate(filterUtils.computeCentroid(append))
        segmentation.makeMovable(sel, t)

        # Hide the old one
        if pointCloudObj.getProperty("Name") in allPointClouds:
            pointCloudObj.setProperty("Visible", False)

    # def onSegmentationEditor():
    #     segmentationpanel.activateSegmentationMode(pointCloudObj.polyData)

    actions = []

    if pointCloudObj:
        actions.extend(
            [
                (None, None),
                ("Copy Pointcloud", onCopyPointCloud),
                #("Merge Pointcloud Into", onMergeIntoPointCloud),
                #("Open Segmentation Editor", onSegmentationEditor),
            ]
        )

    return actions


viewbehaviors.registerContextMenuActions(getRobotActions)


class RobotViewEventFilter(ViewEventFilter):
    # TODO this class taking the class that contains it as a parameter so that it can access its functions is not ideal
    def __init__(self, view):
        super(RobotViewEventFilter, self).__init__(view)

    def onMouseMove(self, event):

        for picker in segmentation.viewPickers:
            if not picker.enabled:
                continue

            picker.onMouseMove(self.getMousePositionInView(event), event.modifiers())
            self.consumeEvent()

    def onLeftMousePress(self, event):
        if event.modifiers() == QtCore.Qt.ControlModifier:
            displayPoint = self.getMousePositionInView(event)
            self.consumeEvent()

        for picker in segmentation.viewPickers:
            if not picker.enabled:
                continue

            picker.onMousePress(self.getMousePositionInView(event), event.modifiers())
            self.consumeEvent()


    def onKeyPress(self, event):

        consumed = False

        key = str(event.text()).lower()

        if key == "r":
            consumed = True
            t = vtk.vtkTransform()
            focalPoint = [0.0, 0.0, 0.25]
            position = [-4.0, -2.0, 2.25]
            t.TransformPoint(focalPoint, focalPoint)
            t.TransformPoint(position, position)
            flyer = cameracontrol.Flyer(self.view)
            flyer.zoomTo(focalPoint, position)

        if key == "t":  # aka 'top'
            consumed = True
            t = vtk.vtkTransform()

            focalPoint = [2, 0.0, 0.25]
            position = [1, 0.0, 15.25]  # to avoid singularities
            t.TransformPoint(focalPoint, focalPoint)
            t.TransformPoint(position, position)
            flyer = cameracontrol.Flyer(self.view)
            flyer.zoomTo(focalPoint, position)

        if consumed:
            self.consumeEvent()
