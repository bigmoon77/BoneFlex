import argparse
import importlib.util
import os
import sys

import bpy


def parse_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    arguments = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    return parser.parse_args(arguments)


def find_skinned_mesh():
    active = bpy.context.view_layer.objects.active
    candidates = []
    if active is not None and active.type == 'MESH':
        candidates.append(active)
    candidates.extend(
        obj for obj in bpy.context.scene.objects
        if obj.type == 'MESH' and obj not in candidates
    )

    for mesh_object in candidates:
        armature_modifier = next(
            (
                modifier for modifier in mesh_object.modifiers
                if modifier.type == 'ARMATURE' and modifier.object is not None
            ),
            None,
        )
        if armature_modifier is not None:
            return mesh_object

    raise RuntimeError(
        "No skinned mesh found. The .blend file must contain a mesh with an Armature modifier."
    )


def load_exporter():
    exporter_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "boneflex_exporter.py")
    spec = importlib.util.spec_from_file_location("boneflex_exporter", exporter_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main():
    args = parse_arguments()
    output_path = os.path.abspath(bpy.path.abspath(args.output))
    os.makedirs(os.path.dirname(output_path), exist_ok=True)

    mesh_object = find_skinned_mesh()
    bpy.ops.object.select_all(action='DESELECT')
    mesh_object.select_set(True)
    bpy.context.view_layer.objects.active = mesh_object

    exporter = load_exporter()
    exporter.register()
    exporter.export_boneflex_asset(bpy.context, output_path, export_glb=False)

    print("BoneFlex .blend import conversion completed:")
    print("  Scene: " + bpy.data.filepath)
    print("  Mesh: " + mesh_object.name)
    print("  Asset: " + output_path)


if __name__ == "__main__":
    main()
