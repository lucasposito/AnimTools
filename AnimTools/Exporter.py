from maya import cmds, mel, OpenMayaUI
import MayaData


def game_exporter(root_jnt='Root_jnt'):
    selected = cmds.ls(sl=True)
    if not selected:
        print('Please select any part of the character you want to export')
        return
    selected_ns = ':'.join(selected[0].split(':')[:-1])

    skeleton_data = MayaData.skeleton.get(f'{selected_ns}:{root_jnt}', False, False)
    if cmds.objExists(skeleton_data['joints'][0]):
        cmds.delete(skeleton_data['joints'][0])

    MayaData.skeleton.load(skeleton_data)

    all_joints = list()
    for jnt in skeleton_data['joints']:
        base = f'{selected_ns}:{jnt}'
        if jnt == root_jnt:
            jnt = f'|{jnt}'
        all_joints.append(jnt)
        cmds.parentConstraint(base, jnt)

    cmds.select(all_joints, r=True)

    if not OpenMayaUI.MQtUtil.findControl('gameExporterWindow'):
        cmds.GameExporterWnd()

    mel.eval('tabLayout -e -sti 2 "gameExporterTabLayout";')  # Change to animation tab

    cmds.setAttr(f'gameExporterPreset2.exportSetIndex', 2)  # Change to export only selected objects

    if not cmds.getAttr('gameExporterPreset2.animClips', mi=1):
        mel.eval('gameExp_AddNewAnimationClip 1;')

    anim_clip = 'gameExporterPreset2.animClips[0]'

    mel.eval('gameExp_DoExport;')
    mel.eval('gameExp_DeleteAnimationClipLayout 0;')

    cmds.delete(all_joints)

