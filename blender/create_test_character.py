# Run inside Blender:
# blender --background --python blender/create_test_character.py -- --output ./test_character.bflex
#
# This creates a small two-bone weighted character and exports both .bflex and .glb.
# It is intended as a reproducible end-to-end smoke-test asset.

import argparse
import importlib.util
import os
import sys

import bpy


def parse_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", default="//test_character.bflex")
    args = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    return parser.parse_args(args)


def clear_scene():
    if bpy.context.object is not None and bpy.context.object.mode != 'OBJECT':
        bpy.ops.object.mode_set(mode='OBJECT')
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)


def create_armature():
    armature_data = bpy.data.armatures.new("BoneFlexTestRig")
    armature_object = bpy.data.objects.new("BoneFlexTestRig", armature_data)
    bpy.context.collection.objects.link(armature_object)

    bpy.context.view_layer.objects.active = armature_object
    armature_object.select_set(True)
    bpy.ops.object.mode_set(mode='EDIT')

    root = armature_data.edit_bones.new("Root")
    root.head = (0.0, 0.0, -1.0)
    root.tail = (0.0, 0.0, 0.0)

    upper = armature_data.edit_bones.new("Upper")
    upper.head = (0.0, 0.0, 0.0)
    upper.tail = (0.0, 0.0, 1.0)
    upper.parent = root
    upper.use_connect = True

    bpy.ops.object.mode_set(mode='OBJECT')
    armature_object.select_set(False)
    return armature_object


def create_weighted_mesh(armature_object):
    bpy.ops.mesh.primitive_uv_sphere_add(
        segments=32,
        ring_count=20,
        radius=1.0,
        location=(0.0, 0.0, 0.0),
    )
    mesh_object = bpy.context.object
    mesh_object.name = "BoneFlexTestCharacter"

    root_group = mesh_object.vertex_groups.new(name="Root")
    upper_group = mesh_object.vertex_groups.new(name="Upper")

    for vertex in mesh_object.data.vertices:
        # Smoothly blend from the lower bone to the upper bone along Blender Z.
        upper_weight = max(0.0, min(1.0, (vertex.co.z + 0.35) / 0.7))
        root_weight = 1.0 - upper_weight
        if root_weight > 1.0e-5:
            root_group.add([vertex.index], root_weight, 'REPLACE')
        if upper_weight > 1.0e-5:
            upper_group.add([vertex.index], upper_weight, 'REPLACE')

    modifier = mesh_object.modifiers.new("BoneFlexArmature", 'ARMATURE')
    modifier.object = armature_object
    mesh_object.parent = armature_object

    bpy.ops.object.select_all(action='DESELECT')
    armature_object.select_set(True)
    mesh_object.select_set(True)
    bpy.context.view_layer.objects.active = mesh_object
    return mesh_object


def load_exporter():
    script_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "boneflex_exporter.py")
    spec = importlib.util.spec_from_file_location("boneflex_exporter", script_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main():
    args = parse_arguments()
    output_path = bpy.path.abspath(args.output)
    if not output_path.lower().endswith(".bflex"):
        output_path += ".bflex"
    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)

    clear_scene()
    armature_object = create_armature()
    create_weighted_mesh(armature_object)

    exporter = load_exporter()
    result_path = exporter.export_boneflex_asset(bpy.context, output_path, export_glb=True)

    print("BoneFlex test character exported:")
    print("  BFLEX: " + output_path)
    print("  GLB:   " + result_path)


if __name__ == "__main__":
    main()
