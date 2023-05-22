from director import visualization as vis
from director.timercallback import TimerCallback
from director import objectmodel as om
from director.debugpolydata import DebugData
from director import ioutils
from PythonQt import QtCore, QtGui, QtUiTools

import os
import numpy as np
from typing import List

class TilePicker():
    def __init__(self, tile_dir, pose_graph_loader):
        self.tile_dir = tile_dir
        self.merged_cloud_filename = "merged_cloud.ply"
        self.merged_cloud = None
        self.pose_graph_loader = pose_graph_loader
        self._load_csv()

    def _load_csv(self):
        filename = os.path.join(self.tile_dir, "tiles.csv")
        self.tiles_data = np.loadtxt(filename, delimiter=",", dtype=np.float64, skiprows=1)

    def load_merged_cloud(self):
        obj = om.findObjectByName("merged_tile_cloud")
        if obj:
            return

        offset = self.pose_graph_loader.offset
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
            obj.setProperty("Alpha", 0.6)

    def load_individual_tile(self, tile_id: int):
        tile_filename = "tile_" + str(tile_id) + ".ply"
        print("Loading", tile_filename)
        offset = self.pose_graph_loader.offset
        path = os.path.join(self.tile_dir, tile_filename)
        poly_data = ioutils.readPolyData(path, ignoreSensorPose=True, offset=offset)

        tile_name = "tile_" + str(tile_id)
        obj = om.findObjectByName(tile_name)
        if obj:
            return

        om.getOrCreateContainer("tiles")
        poly_data = vis.showPolyData(poly_data, tile_name, parent="tiles")
        vis.addChildFrame(poly_data)

    def _is_point_inside_tile(self, x_min: float, y_min: float, size_x: float,
                              size_y: float, x: float, y: float) -> bool:
        return (x >= x_min and x < x_min + size_x and y >= y_min and y < y_min + size_y)

    def find_tile(self, picked_coords: List[float]):
        for tile in self.tiles_data:
            x_utm = tile[1]
            y_utm = tile[2]
            size_x = tile[3]
            size_y = tile[4]
            offset = self.pose_graph_loader.offset
            x = x_utm - offset[0]
            y = y_utm - offset[1]
            if self._is_point_inside_tile(x, y, size_x, size_y, picked_coords[0], picked_coords[1]):
                tile_id = tile[0]
                self.load_individual_tile(int(tile_id))
                return

        print("Cannot find tile", picked_coords)




