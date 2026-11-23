bl_info = {
    "name": "BoneFlex Asset Exporter",
    "author": "BoneFlex",
    "version": (0, 1, 0),
    "blender": (3, 6, 0),
    "location": "View3D > Sidebar > BoneFlex",
    "description": "Export a skinned mesh for the BoneFlex C++ runtime",
    "category": "Import-Export",
}

import bpy
import os
import struct
from bpy_extras.io_utils import ExportHelper
from bpy.props import StringProperty, BoolProperty
from bpy.types import Operator, Panel
from mathutils import Matrix


_AXIS_CONVERSION = Matrix((
    (1.0, 0.0, 0.0, 0.0),
    (0.0, 0.0, 1.0, 0.0),
    (0.0, -1.0, 0.0, 0.0),
    (0.0, 0.0, 0.0, 1.0),
))


def _write_u32(stream, value):
    stream.write(struct.pack("<I", int(value)))


def _write_i32(stream, value):
    stream.write(struct.pack("<i", int(value)))


def _write_f32(stream, value):
    stream.write(struct.pack("<f", float(value)))


def _write_vec3(stream, value):
    _write_f32(stream, value[0])
    _write_f32(stream, value[1])
    _write_f32(stream, value[2])


def _write_quaternion(stream, value):
    _write_f32(stream, value.x)
    _write_f32(stream, value.y)
    _write_f32(stream, value.z)
    _write_f32(stream, value.w)


def _get_active_mesh_and_armature(context):
    mesh_object = context.active_object
    if mesh_object is None or mesh_object.type != 'MESH':
        raise RuntimeError("Select a mesh object as the active object.")

    armature_object = None
    for modifier in mesh_object.modifiers:
        if modifier.type == 'ARMATURE' and modifier.object is not None:
            armature_object = modifier.object
            break

    if armature_object is None:
        raise RuntimeError("The active mesh must have an Armature modifier.")

    if not armature_object.data.bones:
        raise RuntimeError("The Armature contains no bones.")

    return mesh_object, armature_object


def _build_bone_data(armature_object):
    source_bones = list(armature_object.data.bones)
    ordered_bones = []
    visited = set()

    def append_parent_first(bone):
        if bone.name in visited:
            return
        if bone.parent is not None:
            append_parent_first(bone.parent)
        visited.add(bone.name)
        ordered_bones.append(bone)

    for source_bone in source_bones:
        append_parent_first(source_bone)

    bones = ordered_bones
    indices = {bone.name: index for index, bone in enumerate(bones)}
    result = []

    for bone in bones:
        parent_index = indices[bone.parent.name] if bone.parent else -1
        local_matrix = bone.matrix_local.copy()
        if bone.parent:
            local_matrix = bone.parent.matrix_local.inverted() @ local_matrix

        converted = _AXIS_CONVERSION @ local_matrix @ _AXIS_CONVERSION.inverted()
        location, rotation, scale = converted.decompose()
        if any(abs(component - 1.0) > 1.0e-4 for component in scale):
            raise RuntimeError(
                "Bone '{}' has non-unit rest scale. Apply armature/bone scale before export.".format(
                    bone.name
                )
            )
        result.append((bone.name, parent_index, location, rotation.normalized()))

    return result, indices


def _build_mesh_data(mesh_object, armature_object, bone_indices):
    mesh = mesh_object.data
    mesh.calc_loop_triangles()

    mesh_to_armature = armature_object.matrix_world.inverted() @ mesh_object.matrix_world
    conversion = _AXIS_CONVERSION @ mesh_to_armature

    positions = []
    for vertex in mesh.vertices:
        position = conversion @ vertex.co
        positions.append((position.x, position.y, position.z))

    triangle_indices = []
    for triangle in mesh.loop_triangles:
        triangle_indices.extend(triangle.vertices)

    group_to_bone = {}
    for group in mesh_object.vertex_groups:
        if group.name in bone_indices:
            group_to_bone[group.index] = bone_indices[group.name]

    offsets = [0]
    influence_bones = []
    influence_weights = []
    for vertex in mesh.vertices:
        influences = []
        for assignment in vertex.groups:
            bone_index = group_to_bone.get(assignment.group)
            if bone_index is not None and assignment.weight > 0.0:
                influences.append((bone_index, float(assignment.weight)))

        influences.sort(key=lambda item: item[0])
        total_weight = sum(weight for _, weight in influences)
        if total_weight <= 1.0e-8:
            raise RuntimeError(
                "Vertex {} has no positive weight assigned to an exported bone.".format(vertex.index)
            )

        for bone_index, weight in influences:
            influence_bones.append(bone_index)
            influence_weights.append(weight / total_weight)
        offsets.append(len(influence_bones))

    return positions, triangle_indices, offsets, influence_bones, influence_weights


def export_boneflex_asset(context, output_path, export_glb=True):
    mesh_object, armature_object = _get_active_mesh_and_armature(context)
    bone_data, bone_indices = _build_bone_data(armature_object)
    mesh_data = _build_mesh_data(mesh_object, armature_object, bone_indices)

    positions, triangle_indices, offsets, influence_bones, influence_weights = mesh_data
    if not positions or not triangle_indices:
        raise RuntimeError("The active mesh must contain vertices and triangles.")

    output_path = bpy.path.abspath(output_path)
    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)

    with open(output_path, "wb") as stream:
        stream.write(b"BFLX")
        for value in (1, len(positions), len(triangle_indices), len(bone_data), len(influence_bones)):
            _write_u32(stream, value)

        for position in positions:
            _write_vec3(stream, position)

        for index in triangle_indices:
            _write_u32(stream, index)

        for name, parent_index, location, rotation in bone_data:
            encoded_name = name.encode("utf-8")
            _write_u32(stream, len(encoded_name))
            stream.write(encoded_name)
            _write_i32(stream, parent_index)
            _write_vec3(stream, location)
            _write_quaternion(stream, rotation)

        for offset in offsets:
            _write_u32(stream, offset)
        for bone_index in influence_bones:
            _write_u32(stream, bone_index)
        for weight in influence_weights:
            _write_f32(stream, weight)

    glb_path = os.path.splitext(output_path)[0] + ".glb"
    if export_glb:
        if not hasattr(bpy.ops.export_scene, "gltf"):
            raise RuntimeError(
                "The Blender glTF 2.0 exporter is not enabled. Install/enable the glTF exporter "
                "extension, or disable the GLB option to export only the .bflex file."
            )
        selected_objects = list(context.selected_objects)
        active_object = context.view_layer.objects.active
        try:
            bpy.ops.object.select_all(action='DESELECT')
            mesh_object.select_set(True)
            armature_object.select_set(True)
            context.view_layer.objects.active = mesh_object

            available = bpy.ops.export_scene.gltf.get_rna_type().properties.keys()
            kwargs = {
                "filepath": glb_path,
                "export_format": "GLB",
                "use_selection": True,
            }
            if "export_skins" in available:
                kwargs["export_skins"] = True
            if "export_animations" in available:
                kwargs["export_animations"] = True
            bpy.ops.export_scene.gltf(**kwargs)
        finally:
            bpy.ops.object.select_all(action='DESELECT')
            for obj in selected_objects:
                if obj.name in context.view_layer.objects:
                    obj.select_set(True)
            context.view_layer.objects.active = active_object

    return glb_path if export_glb else output_path


class BONEFLEX_OT_export_asset(Operator, ExportHelper):
    bl_idname = "export_scene.boneflex_asset"
    bl_label = "Export BoneFlex Asset"
    bl_options = {'REGISTER'}

    filename_ext = ".bflex"
    filter_glob: StringProperty(default="*.bflex", options={'HIDDEN'})
    export_glb: BoolProperty(
        name="Also export glTF Binary (.glb)",
        description="Export the selected mesh and armature as a standard GLB asset",
        default=True,
    )

    def execute(self, context):
        try:
            result_path = export_boneflex_asset(context, self.filepath, self.export_glb)
        except Exception as error:
            self.report({'ERROR'}, str(error))
            return {'CANCELLED'}

        self.report({'INFO'}, "BoneFlex asset exported: " + result_path)
        return {'FINISHED'}


class BONEFLEX_PT_exporter(Panel):
    bl_label = "BoneFlex"
    bl_idname = "BONEFLEX_PT_exporter"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "BoneFlex"

    def draw(self, context):
        self.layout.operator(BONEFLEX_OT_export_asset.bl_idname, icon='EXPORT')


_classes = (
    BONEFLEX_OT_export_asset,
    BONEFLEX_PT_exporter,
)


def register():
    for cls in _classes:
        bpy.utils.register_class(cls)


def unregister():
    for cls in reversed(_classes):
        bpy.utils.unregister_class(cls)


if __name__ == "__main__":
    register()
