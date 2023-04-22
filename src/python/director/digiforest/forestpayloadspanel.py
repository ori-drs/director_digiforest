import PythonQt
from PythonQt import QtCore, QtGui, QtUiTools
from PyQt5.QtCore import QObject, pyqtSignal, pyqtSlot
from director import applogic as app
from director import transformUtils
from director import objectmodel as om
from director import visualization as vis
from director.thirdparty import transformations
from director.debugpolydata import DebugData
from director import ioutils
from director import filterUtils
from director import vtkNumpy as vnp
from director import applogic as app
from director import segmentation
from director import vtkNumpy
from matplotlib import image as matimage
from director.digiforest.objectpicker import ObjectPicker
from director.digiforest.posegraphloader import PoseGraphLoader

import digiforest_drs as df
import pcl

import os
import re
import numpy as np
import functools
import shutil
import threading

def addWidgetsToDict(widgets, d):

    for widget in widgets:
        if widget.objectName:
            d[str(widget.objectName)] = widget
        addWidgetsToDict(widget.children(), d)


class WidgetDict(object):
    def __init__(self, widgets):
        addWidgetsToDict(widgets, self.__dict__)


class ForestPayloadsPanel(QObject):

    def __init__(self, image_manager):
        QObject.__init__(self)

        loader = QtUiTools.QUiLoader()
        uifile = QtCore.QFile(
            os.path.join(":ui/ddForestPayloads.ui")
        )
        assert uifile.open(uifile.ReadOnly)

        self.widget = loader.load(uifile)
        self.view = app.getDRCView()
        self.ui = WidgetDict(self.widget.children())
        
        self.ui.loadGraphButton.connect(
            "clicked()", self.on_choose_run_input_dir
        )
        self.ui.startPickingButton.connect(
            "clicked()", self._start_node_picking
        )
        self.ui.stopPickingButton.connect(
            "clicked()", self._stop_node_picking
        )
        self.ui.startTreePickingButton.connect(
            "clicked()", self._start_tree_picking
        )
        self.ui.stopTreePickingButton.connect(
            "clicked()", self._stop_node_picking   # not used
        )
        self.ui.generateHeightmaps.connect(
            "clicked()", self.generate_height_maps
        )
        self.ui.loadHeightmaps.connect(
            "clicked()", self.load_all_height_maps
        )
        self.data_dir = None
        self.height_maps_dir_name = "height_maps_in_map"
        self.image_manager = image_manager
        self.tree_data = np.array([])

        
    def run_input_directory(self):
        return os.path.expanduser(self.ui.loadGraphText.text)

    def chooseDirectory(self):
        return QtGui.QFileDialog.getExistingDirectory(
            app.getMainWindow(), "Choose directory...", self.run_input_directory()
        )    

    def get_shorter_name_last(self, name):
        if len(name) > 30:
            name = "..." + name[len(name) - 20 :]

        return name

    def on_choose_run_input_dir(self):
        newDir = self.chooseDirectory()
        if newDir:
            self.data_dir = newDir
            self.ui.loadGraphText.text = self.get_shorter_name_last(newDir)
            self.parse_pose_graph(newDir)
            
    def parse_pose_graph(self, directory):
        self.pose_graph_loader = PoseGraphLoader(directory)
        pose_graph_file = os.path.join(directory, "slam_poses.csv")
        if os.path.isfile(pose_graph_file):
            if not self.pose_graph_loader.load_csv_file(pose_graph_file):
                print("Failed to read data from file: ", pose_graph_file)
                return
        else:
            pose_graph_file = os.path.join(directory, "slam_pose_graph.g2o")
            if not os.path.isfile(pose_graph_file):
                print("Cannot find slam_poses.csv or slam_pose_graph.g2o in", directory)
            else:
                if not self.pose_graph_loader.load_g2o_file(pose_graph_file):
                    print("Failed to read data from file: ", pose_graph_file)
                    return

        # Displaying pose graph data
        colors = [QtGui.QColor(0, 255, 0), QtGui.QColor(255, 0, 0), QtGui.QColor(0, 0, 255),
                  QtGui.QColor(255, 255, 0), QtGui.QColor(255, 0, 255), QtGui.QColor(0, 255, 255)]
        for i in range(0, self.pose_graph_loader.num_experiments):
            exp_num = i+1
            # payload nodes
            obj = vis.showPolyData(
                self.pose_graph_loader.polydata_payloads[i],
                "payload_" + str(exp_num), parent="slam"
            )
            obj.setProperty("Point Size", 8)
            obj.setProperty("Color", colors[(exp_num) % len(colors)])
            vis.addChildFrame(obj)
            # non payload nodes
            obj = vis.showPolyData(
                self.pose_graph_loader.polydata_non_payloads[i],
                "experiment_"+str(exp_num), visible=False, parent="slam"
            )
            obj.setProperty("Point Size", 6)
            obj.setProperty("Color", colors[(exp_num-1) % len(colors)])
            vis.addChildFrame(obj)
            # draw the trajectory
            self._draw_line(self.pose_graph_loader.trajectories[i],
                            name="trajectory_"+str(exp_num), parent="slam")


    def find_tree(self, picked_coords):

        def points_in_cylinder(center, r, length, q):
            '''
            given a cylinder defined by center, length and radius, return whether q is inside it
            '''
            dist = np.linalg.norm(center[0:2]-q[0:2]) # 2d projection
            eps = 0.1
            return (dist <= (r+eps) and np.abs(center[2]-q[2]) <= (0.5*length+eps))


        for tree in self.tree_data:
            axis = tree[3:6]
            center = tree[0:3]
            length = tree[6]
            radius = tree[7]
            q = np.array(picked_coords)
            if points_in_cylinder(center, radius, length, q):
                # found tree
                self.ui.labelLength.text = length
                self.ui.labelRadius.text = radius
                break


    def find_node_data(self, picked_coords):
        '''
        Find and load the data ( point cloud, images ) stored in a node
        '''
        print("LoadPointCloud", picked_coords)
        nodeFound = False
        for row in self.pose_graph_loader.file_data:
            if np.isclose(picked_coords[0], row[3]) and np.isclose(picked_coords[1], row[4]) \
                    and np.isclose(picked_coords[2], row[5]):
                expNum = int(row[0]) // 1000 + 1
                sec = int(row[1])
                nsec = int(row[2])
                trans = (row[3], row[4], row[5])
                quat = (row[9], row[6], row[7], row[8]) # wxyz
                nodeFound = True
                break

        if not nodeFound:
            print("Error : Cannot find picked node")
            return

        local_pointcloud_dir = os.path.join(self.data_dir, "individual_clouds")
        height_map_dir = os.path.join(self.data_dir, self.height_maps_dir_name)
        local_height_map = "height_map_"+str(sec)+"_"+self._convert_nano_secs_to_string(nsec)+".ply"
        height_map_file = os.path.join(height_map_dir, local_height_map)

        local_cloud = "cloud_"+str(sec)+"_"+self._convert_nano_secs_to_string(nsec)+".pcd"
        payload_cloud_dir = os.path.join(self.data_dir, "payload_clouds_in_map")
        tree_description_file = os.path.join(self.data_dir, "trees.csv")

        local_cloud_file = os.path.join(local_pointcloud_dir, local_cloud)
        payload_cloud_file = os.path.join(payload_cloud_dir, local_cloud)
        if os.path.isfile(payload_cloud_file):
            self.load_pointcloud(payload_cloud_file, trans, quat)
        elif os.path.isfile(local_cloud_file):
            self.load_pointcloud(local_cloud_file, trans, quat)

        if os.path.isfile(payload_cloud_file):
            self.terrain_mapping(payload_cloud_file, height_map_file)
        elif os.path.isfile(local_cloud_file):
            self.terrain_mapping(local_cloud_file, height_map_file)

        if os.path.isfile(tree_description_file):
            self.load_cylinders(tree_description_file)

    def load_cylinders(self, filename):
        '''
        From a csv file describing the trees as cylinders, load and display them
        '''
        print("Loading ", filename)
        self.tree_data = np.loadtxt(filename, delimiter=" ", dtype=np.float64)
        id = 0
        for tree in self.tree_data:
            if tree.size < 7:
                continue

            d = DebugData()
            d.addCylinder(center=tree[0:3], axis=tree[3:6], length=tree[6], radius=tree[7])
            polyData = d.getPolyData()
            if not polyData or not polyData.GetNumberOfPoints():
                continue

            # store some information about the trees in the polydata data structure
            length = np.asfortranarray(np.ones(polyData.GetNumberOfPoints())*tree[6])
            vnp.addNumpyToVtk(polyData, length, "length")
            radius = np.asfortranarray(np.ones(polyData.GetNumberOfPoints()) * tree[7])
            vnp.addNumpyToVtk(polyData, radius, "radius")
            # tmp = polyData.GetPointData().GetArray("length")
            # print("tmp", tmp, type(tmp))
            # for i in range(0, tmp.GetNumberOfTuples()):
            #     print(i, tmp.GetTuple(i))
            #

            # print tree info next to the loaded trees
            obj = vis.showPolyData(
                polyData, "tree_"+str(id), parent="trees"
            )
            vis.addChildFrame(obj)

            id += 1

    def load_pointcloud(self, filename, trans, quat):
        print("Loading : ", filename)
        if not os.path.isfile(filename):
            print("File doesn't exist", filename)
            return

        poly_data = ioutils.readPolyData(filename, ignoreSensorPose=True)

        if not poly_data or not poly_data.GetNumberOfPoints():
            print("Error cannot load file")
            return

        #transformedPolyData = self.transformPolyData(polyData, trans, quat)
        cloud_container_name = os.path.splitext(os.path.basename(filename))[0] # removes the extension
        om.getOrCreateContainer(cloud_container_name, parentObj=om.findObjectByName("slam"))
        obj = vis.showPolyData(poly_data, os.path.basename(filename), parent=cloud_container_name)
        vis.addChildFrame(obj)

    # def transformPolyData(self, poly_data, translation, quat):
    #     node_transform = transformUtils.transformFromPose(translation, quat)
    #     transformedPolyData = filterUtils.transformPolyData(poly_data, node_transform)
    #     return transformedPolyData

    def load_all_height_maps(self):
        height_map_dir = os.path.join(self.data_dir, self.height_maps_dir_name)
        if os.path.isdir(height_map_dir):
            height_maps = [f for f in os.listdir(height_map_dir) if os.path.isfile(os.path.join(height_map_dir, f))]
            for height_map_file in height_maps:
                # assuming a file with a name like height_map_1663668471_346755000.ply
                s = re.split('[_ .]', height_map_file)
                if len(s) != 5:
                    continue

                parent_cloud = "cloud_" + s[2] + "_" + s[3]
                self._display_height_map_file(os.path.join(height_map_dir, height_map_file), parent_cloud)
        else:
            print("Cannot read", height_map_dir)


    def convert_heights_mesh(self, parent, height_map_file):
        if not os.path.isfile(height_map_file):
            pcd=pcl.PointCloud()
            pcd.from_list(self.heights_array_raw)
            pcd.to_file(b'/tmp/height_map.pcd')
            os.system("rosrun digiforest_drs generate_mesh") # running a ROS node to convert heights to mesh - nasty!
            height_maps_dir = os.path.dirname(height_map_file)
            if not os.path.isdir(height_maps_dir):
                os.makedirs(height_maps_dir)
            shutil.copyfile('/tmp/height_map.ply', height_map_file)
        else:
            print("Loading height_map", height_map_file)

        self._display_height_map_file(height_map_file, parent)


    def _display_height_map_file(self, height_map_file, parent):
        height_mesh = ioutils.readPolyData(height_map_file)
        height_mesh = segmentation.addCoordArraysToPolyDataXYZ( height_mesh )
        vis.showPolyData(height_mesh, 'Height Mesh', 'Color By', 'z',
                         colorByRange=[self.pose_graph_loader.median_pose_height-4,
                                       self.pose_graph_loader.median_pose_height+4], parent=parent)

    def terrain_mapping(self, filename, height_map_file):
        cloud_pc = pcl.PointCloud_PointNormal()
        cloud_pc._from_pcd_file(filename.encode('utf-8'))
        self._show_pclXYZnormal(cloud_pc, "Cloud Raw", visible=False, parent=os.path.basename(filename))

        # remove non-up points
        cloud = df.filterUpNormal(cloud_pc, 0.95)
        self._show_pclXYZnormal(cloud, "Cloud Up Normals only", visible=False, parent=os.path.basename(filename))

        # drop from xyznormal to xyz
        array_xyz = cloud.to_array()[:, 0:3]
        cloud = pcl.PointCloud()
        cloud.from_array(array_xyz)

        # get the terrain height
        self.heights_array_raw = df.getTerrainHeight(cloud)
        self.convert_heights_mesh(os.path.basename(filename), height_map_file)

        self.heights_pd = vnp.getVtkPolyDataFromNumpyPoints(self.heights_array_raw)
        obj = vis.showPolyData(self.heights_pd, 'Heights', color=[0, 1, 0], visible=False,
                               parent=os.path.basename(filename))
        obj.setProperty('Point Size', 10)

        # filter NaNs *** these could have been removed in function ***
        self.heights_array = self.heights_array_raw[self.heights_array_raw[:, 2] < 10]
        self.heights_pd = vnp.getVtkPolyDataFromNumpyPoints(self.heights_array)
        obj = vis.showPolyData(self.heights_pd, 'Heights Filtered', visible=False, color=[0, 0, 1],
                               parent=os.path.basename(filename))
        obj.setProperty('Point Size', 10)

    def _convert_pcl_to_poly_data(self, cloud):
        polydata=vnp.getVtkPolyDataFromNumpyPoints(cloud.to_array())
        return polydata

    def _show_pclXYZnormal(self, cloud_pc, name, visible, parent):
        array_xyz = cloud_pc.to_array()[:,0:3]
        cloud_pc = pcl.PointCloud()
        cloud_pc.from_array(array_xyz)
        cloud_pd= self._convert_pcl_to_poly_data(cloud_pc)
        vis.showPolyData(cloud_pd, name, visible=visible, parent=parent)

    def _start_node_picking(self, ):
        object_list = [name + str(i) for i in range(1, self.pose_graph_loader.num_experiments+1) for name in ("payload_", "experiment_")]

        picker = ObjectPicker(number_of_points=1, view=app.getCurrentRenderView(), object_list=object_list)
        segmentation.addViewPicker(picker)
        picker.enabled = True
        picker.start()
        picker.annotation_func = functools.partial(self.find_node_data)

    def _start_tree_picking(self, ):
        picker = ObjectPicker(number_of_points=1, view=app.getCurrentRenderView(), pick_type="cells")
        segmentation.addViewPicker(picker)
        picker.enabled = True
        picker.start()
        picker.annotation_func = functools.partial(self.find_tree)

    def _stop_node_picking(self):
        pass

    def generate_height_maps(self):
        #TODO not sure what happens if director is closed while the thread is running
        threading.Thread(target=self._generate_height_maps, daemon=True).start()

    def _generate_height_maps(self):
        height_map_dir = os.path.join(self.data_dir, self.height_maps_dir_name)
        payload_cloud_dir = os.path.join(self.data_dir, "payload_clouds_in_map")
        df.generate_height_maps(payload_cloud_dir, height_map_dir)

    def _convert_nano_secs_to_string(self, nsec):
        """Returns a 9 characters string of a nano sec value"""
        s = str(nsec)
        if len(s) == 9:
            return s

        for i in range(len(s) + 1, 10):
            s = '0' + s
        return s

    def _draw_line(self, points, name, parent):
        d = DebugData()

        for i in range(points.shape[0]-1):
            d.addLine([points[i, 0], points[i, 1], points[i, 2]],
                      [points[i+1, 0], points[i+1, 1], points[i+1, 2]])

        line = vis.updatePolyData(
            d.getPolyData(), name, parent=parent
        )
        line.setProperty("Color", QtGui.QColor(240, 128, 0))
        vis.addChildFrame(line)

    # def _get_image_dir(self, exp_num):
    #     return os.path.join(self.data_dir, "../exp" + str(exp_num), "individual_images")
    #
    # def _get_image_fileName(self, images_dir, sec, nsec):
    #     return os.path.join(images_dir, "image_" + str(sec) + "_" + self._convert_nano_secs_to_string(nsec) + ".png")

    # def _load_image(self, image_file, image_id):
    #     polydata = self._load_image_to_vtk(image_file)
    #     self.image_manager.addImage(image_id, polydata)
    #
    # def _load_image_to_vtk(self, filename):
    #     image_numpy = matimage.imread(filename)
    #     image_numpy = np.multiply(image_numpy, 255).astype(np.uint8)
    #     image_color = self._convert_image_to_color(image_numpy)
    #     image = vtkNumpy.numpyToImageData(image_color)
    #     return image
    #
    # def _convert_image_to_color(self, image):
    #     if len(image.shape) > 2:
    #         return image
    #
    #     color_img = np.zeros((image.shape[0], image.shape[1], 3), dtype=np.uint8)
    #     color_img[:, :, 0] = image
    #     color_img[:, :, 1] = image
    #     color_img[:, :, 2] = image
    #     return color_img


    # def _start_picking(self):
    #     picker = segmentation.PointPicker(numberOfPoints=1, polyDataName='combined_cloud.pcd')
    #     picker.view = app.getDRCView()
    #     segmentation.addViewPicker(picker)
    #     picker.enabled = True
    #     picker.drawLines = False
    #     picker.start()
    #     picker.annotationFunc = functools.partial(self._selectBestImage)


        
def init(image_manager):

    global panels
    global docks

    if "panels" not in globals():
        panels = {}
    if "docks" not in globals():
        docks = {}

    panel = ForestPayloadsPanel(image_manager)
    action = app.addDockAction(
        "ForestPayloadsPanel",
        "Forest Payloads",
        os.path.join(os.path.dirname(__file__), "../images/forest.png"),
    )
    dock = app.addWidgetToDock(
        panel.widget, action=action
    )

    dock.hide()

    return panel
