from maya.api import OpenMaya
from maya import OpenMayaUI, cmds

from AnimTools.pyside import QtWidgets, QtCore, maya_window
from MayaData.lib import constraint, templates

import MayaData
import json
import math

with open(templates.__path__[0] + '\\shapes.json', 'r') as f:
    shapes = json.loads(f.read())


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

    @staticmethod
    def _get_elbow_pos(ik_root, ik_end, pole_vector, upper_length, lower_length):
        root_pos = OpenMaya.MVector(ik_root)
        end_pos = OpenMaya.MVector(ik_end)
        p_vector_pos = OpenMaya.MVector(pole_vector)

        ik_vector = end_pos - root_pos
        ik_length = ik_vector.length()
        ik_dir = ik_vector.normal()

        arm_total = upper_length + lower_length
        if ik_length > arm_total:
            ik_length = arm_total - 0.001  # Slightly shorter to avoid math errors

        # Law of Cosines to find projection length from root to elbow along IK vector
        a = upper_length
        b = lower_length
        c = ik_length

        x = (a ** 2 - b ** 2 + c ** 2) / (2 * c)  # Distance from root along IK line
        proj_point = root_pos + ik_dir * x

        # Get elbow offset direction using pole vector
        pole_dir = p_vector_pos - root_pos
        proj = pole_dir - ik_dir * (pole_dir * ik_dir)
        perp_dir = proj.normal()

        # Elbow height using Pythagoras
        h = math.sqrt(max(0.0, a ** 2 - x ** 2))  # max(0.0, …) guards against negative roots
        return proj_point + perp_dir * h

    @staticmethod
    def _set_aim_vector(main_object, aim_position, sec_position, mirrored=False):
        sec_position = OpenMaya.MVector(sec_position)
        aim_pos = OpenMaya.MVector(aim_position)
        obj_pos = OpenMaya.MVector(cmds.xform(main_object, q=True, ws=True, t=True))

        obj_dag = OpenMaya.MSelectionList().add(main_object).getDagPath(0)
        transform_fn = OpenMaya.MFnTransform(obj_dag)

        aim_axis = OpenMaya.MVector.kXaxisVector
        sec_axis = OpenMaya.MVector.kYaxisVector

        # Construct orthonormal basis
        aim_vector = (aim_pos - obj_pos).normal()
        up_vector = (sec_position - aim_pos).normal()

        if mirrored:
            aim_vector *= -1
            up_vector *= -1

        obj_u = aim_vector  # Forward
        obj_v = up_vector
        obj_w = (obj_u ^ obj_v).normal()  # Cross product → binormal

        obj_v = (obj_w ^ obj_u).normal()  # Recalculate up vector to ensure orthogonality

        # First rotation: align secondary axis to primary axis
        quaternion_u = OpenMaya.MQuaternion(aim_axis, obj_u)
        quaternion = quaternion_u

        # Rotate secondary axis using first quaternion
        sec_axis_rotated = sec_axis.rotateBy(quaternion)

        # Find angle between rotated sec_axis and obj_v (desired up)
        dot = max(min(sec_axis_rotated * obj_v, 1.0), -1.0)  # Clamp dot product
        angle = math.acos(dot)
        quaternion_v = OpenMaya.MQuaternion(angle, obj_u)

        # Fix twist direction if needed
        if not obj_v.isEquivalent(sec_axis_rotated.rotateBy(quaternion_v), 1.0e-5):
            angle = (2 * math.pi) - angle
            quaternion_v = OpenMaya.MQuaternion(angle, obj_u)

        # Combine rotations
        quaternion *= quaternion_v
        transform_fn.setRotation(quaternion, OpenMaya.MSpace.kWorld)

    @staticmethod
    def _get_matrix_vector(matrix, vector):
        vector = vector.lower()
        if vector == 'x':
            return OpenMaya.MVector(matrix[0], matrix[1], matrix[2])
        if vector == 'y':
            return OpenMaya.MVector(matrix[4], matrix[5], matrix[6])
        if vector == 'z':
            return OpenMaya.MVector(matrix[8], matrix[9], matrix[10])

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

        for frame in range(int(self.start_frame), int(self.end_frame) + 1):
            cmds.currentTime(frame, edit=True)
            self.match_ik_to_fk(mods)

    def match_fk_to_ik(self, mods=None):
        if not mods:
            mods = self.check_selection()
        for mod in mods:
            mirrored = False
            if self.modules[mod][0].split('_')[1][0] == 'R':  # Temporary solution to mirrored modules
                mirrored = True
            root_pos = cmds.xform(self.modules[mod][0], q=1, t=1, ws=1)
            end_pos = cmds.xform(self.modules[mod][-1], q=1, t=1, ws=1)
            p_vector_pos = cmds.xform(self.modules[mod][3], q=1, t=1, ws=1)
            upper_length = (OpenMaya.MVector(cmds.xform(self.modules[mod][1], q=1, t=1, ws=1)) -
                            OpenMaya.MVector(cmds.xform(self.modules[mod][0], q=1, t=1, ws=1))).length()

            lower_length = (OpenMaya.MVector(cmds.xform(self.modules[mod][2], q=1, t=1, ws=1)) -
                            OpenMaya.MVector(cmds.xform(self.modules[mod][1], q=1, t=1, ws=1))).length()

            elbow_pos = IKFK()._get_elbow_pos(root_pos, end_pos, p_vector_pos, upper_length, lower_length)

            # Still giving gimbal lock problems
            root_dag = OpenMaya.MSelectionList().add(self.modules[mod][0]).getDagPath(0)
            mat = OpenMaya.MFnTransform(root_dag).getPath().inclusiveMatrix()
            up_vector = IKFK()._get_matrix_vector(mat, 'y') * 10

            IKFK()._set_aim_vector(self.modules[mod][0], elbow_pos, up_vector, mirrored)
            IKFK()._set_aim_vector(self.modules[mod][1], end_pos, p_vector_pos, mirrored)

            cmds.xform(self.modules[mod][1], t=(elbow_pos.x, elbow_pos.y, elbow_pos.z), ws=True)

            # Debug locator at the elbow position
            # cmds.xform(cmds.spaceLocator()[0], t=(elbow_pos.x, elbow_pos.y, elbow_pos.z), ws=True)

    def bake_fk_to_ik(self):
        if not self.modules:
            return

        mods = self.check_selection()
        if not mods:
            mods = list(self.modules.keys())

        for frame in range(int(self.start_frame), int(self.end_frame) + 1):
            cmds.currentTime(frame, edit=True)
            self.match_fk_to_ik(mods)


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

    def __init__(self, parent=maya_window()):
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


class CreateIkFK:
    def __init__(self):
        self.global_ctr = 'global_C0_ctl'
        self.chain_name = 'tentacle'
        self.chain_size = 12

    @staticmethod
    def create_ik(name):
        ik_shape = MayaData.curves.get_shape(f'{name}_ctl')
        ik_color = MayaData.curves.get_color(f'{name}_ctl')
        ik_pos = cmds.xform(f'{name}_ctl', q=True, m=True, ws=True)

        ik_prev = cmds.rename(f'{name}_ctl', f'{name}_prev')
        cmds.setAttr(f'{ik_prev}Shape.visibility', 0)

        ik_ctr = MayaData.curves.load_shape(ik_shape, name=f'{name}_ctl')
        MayaData.curves.load_color(ik_color, ik_ctr)

        ik_offset = cmds.group(ik_ctr, n=f'{name}_offset')
        cmds.xform(ik_offset, m=ik_pos, ws=True)

        cmds.parentConstraint(ik_ctr, ik_prev, skipRotate=['y', 'z'])

        return ik_ctr

    @staticmethod
    def connect_visibility(name, ik_controls, fk_controls, switch_ctr):

        # IK Controls
        for i in ik_controls:
            cmds.connectAttr(f'{switch_ctr}.switchIkFk', f'{i}Shape.visibility')

        # FK Controls
        remap = f'{name}_switch_fk_remap'
        for i in fk_controls:
            if cmds.objExists(remap):
                cmds.connectAttr(f'{remap}.outValue', f'{i}Shape.visibility')
                continue

            remap = cmds.createNode('remapValue', n=remap)
            cmds.connectAttr(f'{switch_ctr}.switchIkFk', f'{remap}.inputValue')
            cmds.setAttr(f'{remap}.outputMin', 1)
            cmds.setAttr(f'{remap}.outputMax', 0)

    def run(self, branch='L0'):
        # Delete global visibility attribute
        for vis_attr in ['chain_IK_vis', 'chain_FK_vis']:
            if cmds.attributeQuery(vis_attr, ex=True, n=self.global_ctr):
                cmds.deleteAttr(f'{self.global_ctr}.{vis_attr}')

        name = f'{self.chain_name}_{branch}'
        ik_name = f'{self.chain_name}_ik_{branch}'

        self.create_ik(f'{ik_name}_ik2')

        # Creating and setting switch control
        switch_ctr = MayaData.curves.load_shape(shapes['knot'], name=f'{name}_switch_ctl')
        MayaData.curves.load_color(22, name=switch_ctr)  # Yellow color

        cmds.group(switch_ctr, n=f'{name}_switch_offset')
        cmds.parentConstraint(f'{name}_{self.chain_size - 1}_jnt', switch_ctr)

        cmds.addAttr(switch_ctr, ln='switchIkFk', at='double', min=0, max=1)
        cmds.setAttr(f'{switch_ctr}.switchIkFk', e=True, keyable=True)

        for attr in ['tx', 'ty', 'tz', 'rx', 'ry', 'rz', 'sx', 'sy', 'sz', 'visibility']:
            cmds.setAttr(f'{switch_ctr}.{attr}', lock=True, keyable=False, channelBox=False)

        # Connecting matrices to blend between IK and FK
        for n_jnt in range(self.chain_size):
            constraint.blend_matrix(f'{name}_{n_jnt}_cns', f'{ik_name}_{n_jnt}_cns', f'{switch_ctr}.switchIkFk')

        # Connecting visibility
        fk_controls = cmds.listRelatives(f'{name}_fk0_npo', ad=True, typ='nurbsCurve')
        fk_controls = set([cmds.listRelatives(child, p=True)[0] for child in fk_controls])
        ik_controls = [f'{ik_name}_ik0_ctl', f'{ik_name}_ik1_ctl', f'{ik_name}_ik2_ctl']

        self.connect_visibility(name, ik_controls, fk_controls, switch_ctr)
