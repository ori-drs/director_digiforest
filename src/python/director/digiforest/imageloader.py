from director import objectmodel as om
from director import visualization as vis
import numpy as np

def _selectBestImage(self, pickedPoint):
    '''
    Givena point, displays the best image stored in the pose graph that can see the point
    '''
    print("Picked point :", pickedPoint)
    combinedPointCloud = om.findObjectByName('combined_cloud.pcd').polyData
    if not combinedPointCloud or not combinedPointCloud.GetNumberOfPoints():
        print("combined_cloud is not found")
        return

    viewpoint = [pickedPoint[0] - self.view.camera().GetPosition()[0],
                 pickedPoint[1] - self.view.camera().GetPosition()[1], 0]
    viewpoint = viewpoint / np.linalg.norm(viewpoint)

    ## browse all nodes
    if self.fileData.size == 0:
        return
    searchRadius = 10
    bestNode = (-1, [0, 0, 0], [0, 0, 0], None, 0, 0)
    for row in self.fileData:
        dist = np.sqrt((pickedPoint[0] - row[3]) ** 2 + (pickedPoint[1] - row[4]) ** 2 + (pickedPoint[2] - row[5]) ** 2)
        if dist < searchRadius:
            expNum = int(row[0]) // 1000 + 1
            sec = int(row[1])
            nsec = int(row[2])
            imagesDir = self._getImageDir(expNum)
            nodePosition = (row[3], row[4], row[5])
            quat = (row[9], row[6], row[7], row[8])  # wxyz
            roll, pitch, yaw = transformUtils.quaternionToRollPitchYaw(quat)
            nodeOrientation = [np.cos(yaw), np.sin(yaw), 0]
            # viewpoint and nodeOrientation must be as aligned as possible
            dotProduct = np.absolute(np.dot(viewpoint, nodeOrientation))
            if dotProduct > bestNode[0] and self._isPointVisible(nodePosition, nodeOrientation, pickedPoint):
                bestNode = (dotProduct, nodeOrientation, nodePosition, imagesDir, sec, nsec)

    # display best images
    if bestNode[3] == None:
        print("Cannot find suitable node")
        return
    self.load_images(bestNode[3], bestNode[4], bestNode[5])

    # debug : visualize best fit
    d = DebugData()
    d.addSphere(bestNode[2], 1.0)
    item = om.findObjectByName("best_node")
    if item:
        om.removeFromObjectModel(item)
    vis.showPolyData(d.getPolyData(), "best_node", parent="slam")
    #
    d = DebugData()
    d.addArrow(pickedPoint, pickedPoint + np.array(viewpoint), 0.3)
    item = om.findObjectByName("viewpoint")
    if item:
        om.removeFromObjectModel(item)
    vis.showPolyData(d.getPolyData(), "viewpoint", parent="slam")
    d = DebugData()
    d.addArrow(bestNode[2], bestNode[2] + np.array(bestNode[1]), 0.3, color=[0, 1, 1])
    item = om.findObjectByName("node_orientation")
    if item:
        om.removeFromObjectModel(item)
    vis.showPolyData(d.getPolyData(), "node_orientation", parent="slam")


def _isPointVisible(self, origin, nodeOrientation, pt):
    '''
    Return true if a point pt can be visible from a node, given the position and orientation of the node
    '''
    if np.dot(nodeOrientation, np.array([pt[0] - origin[0], pt[1] - origin[1], pt[2] - origin[2]])) > 0.3:
        return True
    else:
        return False