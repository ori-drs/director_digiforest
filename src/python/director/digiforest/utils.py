from director import vtkNumpy as vnp
import director.ioutils as io
from PythonQt import QtCore, QtGui, QtUiTools

import pcl
import os
import shutil
import functools
import threading

import time
import numpy as np
from vtk.util.numpy_support import vtk_to_numpy


def convert_nano_secs_to_string(nsec):
    """Returns a 9 characters string of a nano sec value"""
    s = str(nsec)
    if len(s) == 9:
        return s

    for i in range(len(s) + 1, 10):
        s = '0' + s
    return s

def convert_poly_data_to_pcd(poly_data, path: str,
                             output_dir: str):
    '''
    Converts ply file to pcd
    '''
    ext = os.path.splitext(path)[1].lower()
    filename_with_extension = os.path.basename(path)
    filename_without_extension = os.path.splitext(filename_with_extension)[0]
    new_path = os.path.join(output_dir, filename_without_extension) + ".pcd"
    if ext == ".pcd":
        return new_path

    if os.path.isfile(new_path):
        # file already exists
        return new_path

    points = vtk_to_numpy(poly_data.GetPoints().GetData())
    normals = vtk_to_numpy(poly_data.GetPointData().GetNormals())
    zeros_array = np.zeros((points.shape[0], 1))

    # Combine points and normals into a single NumPy array
    array = np.concatenate((points, normals, zeros_array), axis=-1)
    array = array.astype(np.float32)

    cloud = pcl.PointCloud_PointNormal()
    cloud.from_array(array)

    pcl.save_PointNormal(cloud, "/tmp/cloud.pcd")
    if not os.path.isdir(output_dir):
        os.makedirs(output_dir)
    shutil.copyfile("/tmp/cloud.pcd", new_path)
    return new_path

def convert_heights_mesh(heights_array_raw, height_map_file: str):
    if not os.path.isfile(height_map_file):
        pcd = pcl.PointCloud()
        pcd.from_list(heights_array_raw)
        pcd.to_file(b'/tmp/height_map.pcd')
        os.system("rosrun digiforest_drs generate_mesh") # running a ROS node to convert heights to mesh - nasty!
        height_maps_dir = os.path.dirname(height_map_file)
        if not os.path.isdir(height_maps_dir):
            os.makedirs(height_maps_dir)
        shutil.copyfile('/tmp/height_map.ply', height_map_file)
    else:
        print("Loading height_map", height_map_file)

def loading_popup(popup_message: str):
    '''Decorator to show a popup window before a function call.
       The window is closed after the function returned'''
    def wrap(func):
        @functools.wraps(func)
        def wrapped_func(*args, **kwargs):

            message_box = QtGui.QMessageBox()
            message_box.setIcon(QtGui.QMessageBox.Information)
            message_box.setText(popup_message)
            message_box.setWindowTitle("Please Wait")
            message_box.setStandardButtons(QtGui.QMessageBox.NoButton)
            message_box.show()
            time.sleep(0.2)
            QtCore.QCoreApplication.instance().processEvents()

            #func(*args, **kwargs)
            # run the function inside a thread
            thread = threading.Thread(target=func, args=args, kwargs=kwargs)
            thread.start()

            while thread.is_alive():
                time.sleep(0.2)
                QtCore.QCoreApplication.instance().processEvents()

            message_box.accept()  # closing message box
        return wrapped_func
    return wrap
