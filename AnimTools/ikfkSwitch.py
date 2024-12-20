from maya.api import OpenMaya
from maya import OpenMayaUI, cmds

from PySide2 import QtWidgets
from PySide2 import QtCore
from shiboken2 import wrapInstance

from MayaData.lib import constraint


def maya_main_window():
    main_window_ptr = OpenMayaUI.MQtUtil.mainWindow()
    return wrapInstance(int(main_window_ptr), QtWidgets.QWidget)


def retrieve_fk_pose(hierarchy):
    parent_mat = None
    for ctr in hierarchy:
        ctr_obj = OpenMaya.MSelectionList().add(ctr).getDependNode(0)
        parent_obj = OpenMaya.MFnTransform(ctr_obj).parent(0)

        if not parent_mat:
            parent_mat = OpenMaya.MFnDagNode(parent_obj).getPath().inclusiveMatrix()  # world matrix
            continue
        current_parent_mat = OpenMaya.MFnTransform(parent_obj).transformation().asMatrix()  # local matrix
        parent_mat = current_parent_mat * parent_mat

    return parent_mat


class IKFK(object):
    def __init__(self):
        # New Name order: Limb _ Side(Number) _ type _ Suffix

        self.limb = ['arm', 'leg']
        self.side = ['L', 'R', 'C']
        self.fk_mod = ['fk0', 'fk1', 'fk2']
        self.ik_mod = ['upv', 'ik']
        self.ctr_suffix = 'ctl'

        self.start_frame = None
        self.end_frame = None

        self.modules = dict()
        self.selection = dict()

    def check_selection(self):
        selected = list()

        for each in cmds.ls(sl=True):
            if each not in self.selection:
                continue
            if self.selection[each] not in selected:
                selected.append(self.selection[each])

        if not selected:
            return self.modules.keys()
        return selected

    def match_tip(self, module, fk=False):
        driver, driven = (2, 4) if not fk else (4, 2)

        ik_obj = OpenMaya.MSelectionList().add(self.modules[module][-1]).getDependNode(0)
        ik_obj = OpenMaya.MFnTransform(ik_obj).parent(0)

        constraint.matrix(self.modules[module][driver], self.modules[module][driven])

        neutral_mat = {2: retrieve_fk_pose(self.modules[module][:3]),
                       4: OpenMaya.MFnDagNode(ik_obj).getPath().inclusiveMatrix()}

        offset_mat = neutral_mat[driven] * neutral_mat[driver].inverse()

        driven_obj = OpenMaya.MSelectionList().add(self.modules[module][driven]).getDependNode(0)
        driven_mat = OpenMaya.MFnDagNode(driven_obj).getPath().inclusiveMatrix()

        cmds.xform(self.modules[module][driven], m=offset_mat * driven_mat, ws=True)

    def _pole_vector(self, root, mid, end):
        point_a = OpenMaya.MVector(*root)
        point_b = OpenMaya.MVector(*mid)
        point_c = OpenMaya.MVector(*end)

        vector_ab = point_b - point_a
        vector_ac = point_c - point_a
        ac_normal = vector_ac.normalize()

        proj_length = vector_ab * ac_normal
        proj_vector = (ac_normal * proj_length) + point_a

        vector_pb = point_b - proj_vector
        pb_normal = vector_pb.normalize()
        pole_position = point_b + (pb_normal * vector_ab.length())
        return pole_position.x, pole_position.y, pole_position.z

    def match_ik_to_fk(self, mods=None):
        if not mods:
            mods = self.check_selection()

        for mod in mods:
            # Query FK position
            root = cmds.xform(self.modules[mod][0], q=True, ws=True, t=True)
            mid = cmds.xform(self.modules[mod][1], q=True, ws=True, t=True)
            tip = cmds.xform(self.modules[mod][2], q=True, ws=True, t=True)

            self.match_tip(mod)
            cmds.xform(self.modules[mod][3], ws=True, t=self._pole_vector(root, mid, tip))

    def bake_ik_to_fk(self):
        if not self.modules:
            return

        mods = self.check_selection()
        if not mods:
            mods = list(self.modules.keys())

        playback_start = self.start_frame
        playback_end = self.end_frame
        timeline = range(int(playback_start), int(playback_end))
        controllers = list()

        for key in mods:
            controllers.append(self.modules[key][3])
            controllers.append(self.modules[key][4])

        for frame in timeline:
            cmds.currentTime(frame, edit=True)

            cmds.cutKey(controllers, time=(frame, frame), option="keys")
            self.match_ik_to_fk(mods)
            cmds.setKeyframe(controllers, hi='none', at=['translate', 'rotate'], s=False, t=frame)
        cmds.delete(controllers, sc=True)

    def match_fk_to_ik(self, mods=None):
        pass
        # if not mods:
        #     mods = self.check_selection()
        #
        # for mod in mods:
        #     root_ik = '_'.join(self.modules[mod][2].split('_')[:-1] + [self.ik_suffix])
        #     mid_ik = '_'.join(self.modules[mod][1].split('_')[:-1] + [self.ik_suffix])
        #
        #     constraint.matrix(root_ik, '{}_{}'.format(self.modules[mod][2], self.ctr_suffix))
        #     constraint.matrix(mid_ik, '{}_{}'.format(self.modules[mod][1], self.ctr_suffix))
        #     self.match_tip(mod, fk=True)

    def bake_fk_to_ik(self):
        if not self.modules:
            return

        mods = self.check_selection()
        if not mods:
            mods = list(self.modules.keys())

        playback_start = self.start_frame
        playback_end = self.end_frame
        timeline = range(int(playback_start), int(playback_end))
        controllers = []
        for key in mods:
            controllers.append('{}_{}'.format(self.modules[key][0], self.ctr_suffix))
            controllers.append('{}_{}'.format(self.modules[key][1], self.ctr_suffix))
            controllers.append('{}_{}'.format(self.modules[key][2], self.ctr_suffix))
        for frame in timeline:
            cmds.currentTime(frame, edit=True)
            cmds.cutKey(controllers, time=(frame, frame), option="keys")

            self.match_fk_to_ik(mods)
            cmds.setKeyframe(controllers, hi='none', at=['translate', 'rotate'], s=False, t=frame)
        cmds.delete(controllers, sc=True)


class ikfkUI(QtWidgets.QDialog):
    ui_instance = None

    @classmethod
    def show_ui(cls):
        if not cls.ui_instance:
            cls.ui_instance = ikfkUI()

        if cls.ui_instance.isHidden():
            cls.ui_instance.show()
        else:
            cls.ui_instance.raise_()
            cls.ui_instance.activateWindow()

    def __init__(self, parent=maya_main_window()):
        super(ikfkUI, self).__init__(parent)

        self.setWindowTitle("IK FK Switch")
        self.ik_fk = IKFK()
        self.width = 280
        self.setMinimumWidth(self.width)
        self.setMaximumWidth(self.width + 10)
        self.setMinimumHeight(self.width)

        self.setWindowFlags(self.windowFlags() ^ QtCore.Qt.WindowContextHelpButtonHint)

        self.start_frame = int(cmds.playbackOptions(q=True, minTime=True))
        self.ik_fk.start_frame = self.start_frame
        self.end_frame = int(cmds.playbackOptions(q=True, maxTime=True))
        self.ik_fk.end_frame = self.end_frame

        self.create_widgets()
        self.create_layouts()
        self.create_connections()

    def closeEvent(self, event):
        self.clear_all_modules()

    def create_widgets(self):

        self.table = QtWidgets.QTableWidget()
        self.table.setColumnCount(1)
        self.table.setHorizontalHeaderLabels(['Modules Detected'])
        header_view = self.table.horizontalHeader()
        header_view.setSectionResizeMode(0, QtWidgets.QHeaderView.Stretch)

        self.add_button = QtWidgets.QPushButton('ADD')
        self.add_button.setMaximumWidth(70)

        self.start_frame_field = QtWidgets.QLineEdit(str(self.start_frame))
        self.start_frame_field.setMaximumWidth(60)

        self.end_frame_field = QtWidgets.QLineEdit(str(self.end_frame))
        self.end_frame_field.setMaximumWidth(60)

        # generate button
        self.match_ikfk_button = QtWidgets.QPushButton('SNAP IK TO FK')
        self.match_ikfk_button.setMinimumWidth((self.width - 20) / 2)
        self.match_ikfk_button.setMinimumHeight(40)

        self.match_fkik_button = QtWidgets.QPushButton('SNAP FK TO IK')
        self.match_fkik_button.setMinimumWidth((self.width - 20) / 2)
        self.match_fkik_button.setMinimumHeight(40)

        self.bake_ikfk_button = QtWidgets.QPushButton('IK FOLLOWS FK')
        self.bake_ikfk_button.setMinimumWidth((self.width - 20) / 2)
        self.bake_ikfk_button.setMinimumHeight(40)

        self.bake_fkik_button = QtWidgets.QPushButton('FK FOLLOWS IK')
        self.bake_fkik_button.setMinimumWidth((self.width - 20) / 2)
        self.bake_fkik_button.setMinimumHeight(40)

    def create_layouts(self):
        frame_layout = QtWidgets.QHBoxLayout()
        frame_layout.addWidget(self.start_frame_field)
        frame_layout.addWidget(self.end_frame_field)

        match_layout = QtWidgets.QHBoxLayout()
        match_layout.addStretch()
        match_layout.addWidget(self.match_ikfk_button)
        match_layout.addWidget(self.match_fkik_button)

        bake_layout = QtWidgets.QHBoxLayout()
        bake_layout.addStretch()
        bake_layout.addWidget(self.bake_ikfk_button)
        bake_layout.addWidget(self.bake_fkik_button)

        header_layout = QtWidgets.QHBoxLayout()

        extra_buttons_layout = QtWidgets.QHBoxLayout()
        extra_buttons_layout.addStretch()
        extra_buttons_layout.addWidget(self.add_button)

        main_layout = QtWidgets.QVBoxLayout(self)
        main_layout.addLayout(header_layout)
        main_layout.addWidget(self.table)
        main_layout.addLayout(extra_buttons_layout)
        main_layout.addLayout(frame_layout)
        main_layout.addLayout(match_layout)
        main_layout.addLayout(bake_layout)

    def create_connections(self):

        self.table.itemSelectionChanged.connect(self.select_items)

        self.add_button.clicked.connect(self.add)

        self.start_frame_field.textChanged.connect(self.set_start_frame)
        self.end_frame_field.textChanged.connect(self.set_end_frame)

        self.match_ikfk_button.clicked.connect(self.ik_fk.match_ik_to_fk)
        self.match_fkik_button.clicked.connect(self.ik_fk.match_fk_to_ik)
        self.bake_ikfk_button.clicked.connect(self.ik_fk.bake_ik_to_fk)
        self.bake_fkik_button.clicked.connect(self.ik_fk.bake_fk_to_ik)

    def set_start_frame(self):
        self.ik_fk.start_frame = int(self.start_frame_field.text())

    def set_end_frame(self):
        self.ik_fk.end_frame = int(self.end_frame_field.text())

    def clear_all_modules(self):
        self.ik_fk.modules = dict()
        self.ik_fk.selection = dict()

        self.table.clear()
        self.table.setRowCount(0)
        self.table.setHorizontalHeaderLabels(['Modules Detected'])

    def select_items(self):
        items = self.table.selectedItems()
        names = list()
        for item in items:
            names.append(self.ik_fk.modules[item.text()][0])
        cmds.select(names, r=True)

    def add(self):
        namespace = list()

        for selected in cmds.ls(sl=1):
            ns = cmds.ls(selected, sns=1)[-1]
            if ns != ':' and ns not in namespace:
                namespace.append(ns)

        if not namespace:
            self.add_modules()
            return

        for ns in namespace:
            self.add_modules(ns)

    def add_modules(self, namespace=None):
        self.clear_all_modules()

        for sid in self.ik_fk.side:
            arm, leg = self.ik_fk.limb
            if namespace:
                arm_mods = f'{namespace}:{arm}_{sid}'
                leg_mods = f'{namespace}:{leg}_{sid}'
            else:
                arm_mods = f'{arm}_{sid}'
                leg_mods = f'{leg}_{sid}'

            arm_controls = cmds.ls(f'{arm_mods}*{self.ik_fk.ctr_suffix}')
            number_arm_mods = {int(item.partition(arm_mods)[-1].split('_')[0]) for item in arm_controls}

            for n in range(len(number_arm_mods)):
                arm_fk = [f'{arm_mods}{n}_{fk}_{self.ik_fk.ctr_suffix}' for fk in self.ik_fk.fk_mod]
                arm_ik = [f'{arm_mods}{n}_{ik}_{self.ik_fk.ctr_suffix}' for ik in self.ik_fk.ik_mod]
                self.insert_item(n, 0, f'{arm_mods}{n}', arm_fk + arm_ik)

            leg_controls = cmds.ls(f'{leg_mods}*{self.ik_fk.ctr_suffix}')
            number_leg_mods = {int(item.partition(leg_mods)[-1].split('_')[0]) for item in leg_controls}

            for n in range(len(number_leg_mods)):
                leg_fk = [f'{leg_mods}{n}_{fk}_{self.ik_fk.ctr_suffix}' for fk in self.ik_fk.fk_mod]
                leg_ik = [f'{leg_mods}{n}_{ik}_{self.ik_fk.ctr_suffix}' for ik in self.ik_fk.ik_mod]
                self.insert_item(n, 0, f'{leg_mods}{n}', leg_fk + leg_ik)

    def insert_item(self, row, column, text, module):
        self.ik_fk.modules[text] = module
        self.table.insertRow(row)
        item = QtWidgets.QTableWidgetItem(text)
        self.table.setItem(row, column, item)

        for mod in module:
            self.ik_fk.selection[mod] = text
