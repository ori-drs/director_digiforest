import os

from PythonQt import QtCore, QtGui

import director.applogic as app
import director.objectmodel as om
import director.ioutils as io
import director.visualization as vis

_lastDir = None


def getDefaultDirectory():
    return _lastDir or os.getcwd()


def storeDefaultDirectory(filename):

    global _lastDir
    if os.path.isfile(filename):
        filename = os.path.dirname(filename)
    if os.path.isdir(filename):
        _lastDir = filename


def onFileOpenDialog():

    mainWindow = app.getMainWindow()

    fileFilters = "Data Files (*.obj *.pcd *.ply *.stl *.vtk *.vtp *.wrl *.urdf *.otdf)"
    filename = QtGui.QFileDialog.getOpenFileName(
        mainWindow, "Open...", getDefaultDirectory(), fileFilters
    )
    if not filename:
        return

    storeDefaultDirectory(filename)
    onOpenFile(filename)


def onOpenFile(filename):

    onOpenGeometry(filename)


def onOpenGeometry(filename):

    if filename.lower().endswith("wrl"):
        onOpenVrml(filename)
        return

    polyData = io.readPolyData(filename)

    if not polyData or not polyData.GetNumberOfPoints():
        app.showErrorMessage(
            "Failed to read any data from file: %s" % filename, title="Reader error"
        )
        return

    vis.showPolyData(polyData, os.path.basename(filename), parent="files")


def onOpenVrml(filename):
    meshes, color = io.readVrml(filename)
    folder = om.getOrCreateContainer(
        os.path.basename(filename), parentObj=om.getOrCreateContainer("files")
    )
    for i, pair in enumerate(zip(meshes, color)):
        mesh, color = pair
        vis.showPolyData(mesh, "mesh %d" % i, color=color, parent=folder)



def onFileSaveData():

    obj = om.getActiveObject()
    if not obj:
        app.showErrorMessage("Please select an object", title="No object selected")
        return
    if hasattr(obj, "polyData"):
        mainWindow = app.getMainWindow()
        fileFilters = "PLY (*.ply);;STL (*.stl);;VTP (*.vtp);;VTK (*.vtk)"
        defaultFilter = "VTP (*.vtp)"
        filename = QtGui.QFileDialog.getSaveFileName(
            mainWindow,
            "Save Data...",
            getDefaultDirectory(),
            fileFilters,
            defaultFilter,
        )

        if not filename:
            return

        if not os.path.splitext(filename)[1]:
            filename += ".vtp"

        polyData = io.writePolyData(obj.polyData, filename)
    else:
        app.showErrorMessage(
            "Please select an object that contains geometry data or an OTDF object",
            title="Invalid object selected",
        )
        return

    storeDefaultDirectory(filename)


def onOpenOnlineHelp():

    QtGui.QDesktopServices.openUrl(
        QtCore.QUrl("https://openhumanoids.github.io/director/")
    )


def init():
    mainWindow = app.getMainWindow()

    mainWindow.connect("fileOpen()", onFileOpenDialog)
    mainWindow.connect("fileSaveData()", onFileSaveData)
    mainWindow.connect("openOnlineHelp()", onOpenOnlineHelp)
