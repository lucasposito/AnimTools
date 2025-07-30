from maya import cmds
from maya import OpenMayaUI

version = int(cmds.about(version=True))

if version <= 2024:
    from PySide2 import QtWidgets, QtCore, QtGui
    from shiboken2 import wrapInstance
    # Launching MayaData 2024 and earlier
else:
    from PySide6 import QtWidgets, QtCore, QtGui
    from shiboken6 import wrapInstance
    # Launching MayaData 2025 and later

def maya_window():
    ptr = OpenMayaUI.MQtUtil.mainWindow()
    return wrapInstance(int(ptr), QtWidgets.QWidget)
