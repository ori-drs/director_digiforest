from director import visualization as vis
from director.timercallback import TimerCallback
from director import objectmodel as om
from director.debugpolydata import DebugData
from PythonQt import QtCore, QtGui, QtUiTools

class ObjectPicker(TimerCallback):
    def __init__(self, view, pick_type="points", number_of_points=3, object_list=None):
        TimerCallback.__init__(self)
        self.targetFps = 30
        self.enabled = False
        self.pick_type = pick_type
        self.number_of_points = number_of_points
        self.annotation_obj = None
        self.object_list = object_list
        self.view = view
        self.clear()

    def clear(self):
        self.points = [None for i in range(self.number_of_points)]
        self.hover_pos = None
        self.annotation_func = None
        self.last_move_pos = [0, 0]

    def onMouseMove(self, displayPoint, modifiers=None):
        self.last_move_pos = displayPoint

    def onMousePress(self, displayPoint, modifiers=None):
        for i in range(self.number_of_points):
            if self.points[i] is None:
                self.points[i] = self.hover_pos
                break

        if self.points[-1] is not None:
            self.finish()

    def finish(self):

        self.enabled = False
        om.removeFromObjectModel(self.annotation_obj)

        points = [p.copy() for p in self.points]
        if self.annotation_func is not None:
            self.annotation_func(*points)

    def handleRelease(self, displayPoint):
        pass

    def draw(self):

        d = DebugData()

        points = [p if p is not None else self.hover_pos for p in self.points]

        # draw points
        for p in points:
            if p is not None:
                d.addSphere(p, radius=0.08)

        self.annotation_obj = vis.updatePolyData(
            d.getPolyData(), "annotation", parent=om.findObjectByName("slam")
        )
        self.annotation_obj.setProperty("Color", QtGui.QColor(0, 255, 0))
        self.annotation_obj.actor.SetPickable(False)

    def tick(self):
        if not self.enabled:
            return

        if self.object_list is not None:
            for object_name in self.object_list:
                picked_point_fields = vis.pickPoint(
                    self.last_move_pos, self.view, pickType=self.pick_type, obj=object_name
                )
                if picked_point_fields.pickedDataset is not None:
                    self.hover_pos = picked_point_fields.pickedPoint
                    self.draw()
                    return
        else:
            picked_point_fields = vis.pickPoint(
                self.last_move_pos, self.view, pickType=self.pick_type
            )
            self.hover_pos = picked_point_fields.pickedPoint
            self.draw()