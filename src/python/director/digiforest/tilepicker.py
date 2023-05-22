from director import visualization as vis
from director.timercallback import TimerCallback
from director import objectmodel as om
from director.debugpolydata import DebugData
from director import ioutils
from PythonQt import QtCore, QtGui, QtUiTools

import os

class TilePicker():
    def __init__(self, tile_dir, pose_graph_loader):
        self.tile_dir = tile_dir
        self.merged_cloud_filename = "merged_cloud.ply"
        self.merged_cloud = None
        self.pose_graph_loader = pose_graph_loader

    def load_merged_cloud(self):

        offset = self.pose_graph_loader.first_node_position(exp_num=1)
        merge_cloud_path = os.path.join(self.tile_dir, self.merged_cloud_filename)
        poly_data = ioutils.readPolyData(merge_cloud_path, ignoreSensorPose=True, offset=offset)

        om.getOrCreateContainer("tiles")
        self.merged_cloud = vis.showPolyData(poly_data, "merged_tile_cloud", parent="tiles")
        vis.addChildFrame(self.merged_cloud)

    def display_merged_cloud(self, visible=True):
        if self.merged_cloud is None:
            self.load_merged_cloud()

        obj = om.findObjectByName("merged_tile_cloud")
        if obj:
            obj.setProperty("Visible", visible)


