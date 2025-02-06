
from maya import OpenMayaUI, cmds

from PySide2 import QtWidgets
from PySide2 import QtCore
from shiboken2 import wrapInstance

import json
from pathlib import Path


def maya_main_window():
    main_window_ptr = OpenMayaUI.MQtUtil.mainWindow()
    return wrapInstance(int(main_window_ptr), QtWidgets.QWidget)


class NameUI(QtWidgets.QDialog):
    custom_signal = QtCore.Signal(str)

    def __init__(self, parent=None):
        super(NameUI, self).__init__(parent)

        self.setWindowTitle('Selection Name')
        self.setWindowFlags(self.windowFlags() ^ QtCore.Qt.WindowContextHelpButtonHint)

        self.setMinimumWidth(280)

        self.create_layout()

    def create_layout(self):
        line_edit = QtWidgets.QLineEdit()
        line_edit.returnPressed.connect(lambda: self.send_signal(line_edit.text()))

        main_layout = QtWidgets.QVBoxLayout(self)
        main_layout.addWidget(line_edit)

    def send_signal(self, name):
        self.custom_signal.emit(name)
        self.close()


class SelectUI(QtWidgets.QDialog):
    ui_instance = None

    @classmethod
    def show_ui(cls):
        if not cls.ui_instance:
            cls.ui_instance = SelectUI()

        if cls.ui_instance.isHidden():
            cls.ui_instance.show()
        else:
            cls.ui_instance.raise_()
            cls.ui_instance.activateWindow()
        cls.ui_instance.update_data(cls.ui_instance)
        cls.ui_instance.update_namespaces(cls.ui_instance)

    def __init__(self, parent=maya_main_window()):
        super(SelectUI, self).__init__(parent)

        self.setWindowTitle("Selection Helper")
        self.width = 280
        self.setMinimumWidth(self.width)
        self.setMaximumWidth(self.width + 10)
        self.setMinimumHeight(self.width)

        self.setWindowFlags(self.windowFlags() ^ QtCore.Qt.WindowContextHelpButtonHint)

        self.selection_data = dict()

        self.create_widgets()
        self.create_layouts()
        self.create_connections()

    def keyPressEvent(self, event):
        if event.key() == QtCore.Qt.Key_Delete:
            self.delete_selection()
    
    def contextMenuEvent(self, event):
        position = self.base_table.mapFromGlobal(event.globalPos())

        popup_menu = QtWidgets.QMenu(self)
        
        if self.base_table.itemAt(position):
            selected_item = self.base_table.selectedItems()[0]
            popup_menu.addAction('Rename', lambda: self.rename_selection(selected_item))
            popup_menu.exec_(event.globalPos())
            return
        
        popup_menu.addAction('Add Selection', self.handle_signal)
        popup_menu.exec_(event.globalPos())

    @staticmethod
    def get_selection():
        return [one.split(':')[-1] for one in cmds.ls(sl=True)]

    @staticmethod
    def data_path():
        file_path = Path(__file__).parent / 'SelectionData.json'
        if not file_path.exists():
            file_path.touch()
        return str(file_path)

    @staticmethod
    def update_data(instance):
        with open(SelectUI.data_path(), 'r') as f:
            content = f.read().strip()
            if content:
                instance.selection_data = json.loads(content)
                instance.refresh()

    @staticmethod
    def update_namespaces(instance):
        instance.namespace_box.clear()
        namespaces = [name for name in cmds.namespaceInfo(lon=True, r=True) if name not in ['UI', 'shared']] + ['None']
        for name in namespaces:
            instance.namespace_box.addItem(name)
        instance.namespace_box.setCurrentText('None')

    def create_widgets(self):
        self.base_table = QtWidgets.QTableWidget()
        self.base_table.setColumnCount(1)
        self.base_table.horizontalHeader().setVisible(False)
        self.base_table.verticalHeader().setVisible(False)

        self.base_table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)

        self.namespace_box = QtWidgets.QComboBox()
        SelectUI.update_namespaces(self)

    def create_layouts(self):
        main_layout = QtWidgets.QVBoxLayout(self)
        main_layout.addWidget(self.base_table)
        main_layout.addWidget(self.namespace_box)

    def create_connections(self):
        self.base_table.itemDoubleClicked.connect(self.load_selection)

    def get_items(self):
        return [self.base_table.item(row, 0) for row in range(self.base_table.rowCount())]
    
    def refresh(self):
        current_items = [item.text() for item in self.get_items()]
        for name, data in self.selection_data.items():
            if name not in current_items:
                self.new_selection(name, data)

    def handle_signal(self):
        name_dialog = NameUI()
        name_dialog.custom_signal.connect(lambda name: self.new_selection(name))
        name_dialog.show()
    
    def new_selection(self, name, selection=None):
        
        row = self.base_table.rowCount()
        self.base_table.insertRow(row)

        if not selection:
            selection = SelectUI.get_selection()
        self.selection_data[name] = selection

        with open(SelectUI.data_path(), 'w') as f:
            f.write(json.dumps(self.selection_data, indent=4))

        item = QtWidgets.QTableWidgetItem(name)
        item.setData(QtCore.Qt.UserRole, selection)
        item.setFlags(QtCore.Qt.ItemIsSelectable | QtCore.Qt.ItemIsEnabled)
    
        self.base_table.setItem(row, 0, item)

    def load_selection(self, item):
        selection_data = item.data(QtCore.Qt.UserRole)

        namespace = self.namespace_box.currentText()
        if namespace != 'None':
            selection_data = [f'{namespace}:{one}' for one in selection_data]

        cmds.select(selection_data, r=True)

    def rename_selection(self, item):
        item.setFlags(item.flags() | QtCore.Qt.ItemIsEditable)
        self.base_table.editItem(item)
        item.setFlags(QtCore.Qt.ItemIsSelectable | QtCore.Qt.ItemIsEnabled)
        # TODO: Work on Rename method

    def delete_selection(self):
        selected_rows = sorted(set(item.row() for item in self.base_table.selectedItems()), reverse=True)

        for row in selected_rows:
            name = self.base_table.item(row, 0).text()
            if name in self.selection_data:
                del self.selection_data[name]
            self.base_table.removeRow(row)

        with open(SelectUI.data_path(), 'w') as f:
            f.write(json.dumps(self.selection_data, indent=4))
