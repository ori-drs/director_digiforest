from PythonQt import QtCore, QtGui, QtUiTools
import os
from . import vtkAll as vtk
from .shallowCopy import shallowCopy


class PointCloudOffsetInputDialog(QtGui.QDialog):
    def __init__(self, offset_x: float, offset_y: float, offset_z: float, parent=None):
        super().__init__(parent)

        button_box = QtGui.QDialogButtonBox(QtGui.QDialogButtonBox.Ok | QtGui.QDialogButtonBox.Cancel, self)
        layout = QtGui.QFormLayout(self)

        self.inputs = []
        layout.addRow(QtGui.QLabel("Coordinates are too big, choose an offset to avoid losing precision "))

        validator = QtGui.QDoubleValidator()
        validator.setDecimals(2)

        line_edit_x = QtGui.QLineEdit(str(offset_x), self)
        line_edit_x.setValidator(validator)
        self.inputs.append(line_edit_x)
        layout.addRow("offset_x", self.inputs[-1])
        line_edit_y = QtGui.QLineEdit(str(offset_y), self)
        line_edit_y.setValidator(validator)
        self.inputs.append(line_edit_y)
        layout.addRow("offset_y", self.inputs[-1])
        line_edit_z = QtGui.QLineEdit(str(offset_z), self)
        line_edit_z.setValidator(validator)
        self.inputs.append(line_edit_z)
        layout.addRow("offset_z", self.inputs[-1])

        layout.addWidget(button_box)

        button_box.accepted.connect(self.accept)
        button_box.rejected.connect(self.reject)

    def get_offset_x(self) -> float:
        return float(self.inputs[0].text)

    def get_offset_y(self) -> float:
        return float(self.inputs[1].text)

    def get_offset_z(self) -> float:
        return float(self.inputs[2].text)
