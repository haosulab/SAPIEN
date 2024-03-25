import bpy
import sys


def clear_scene():
    bpy.ops.object.mode_set(mode="OBJECT")
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    bpy.ops.outliner.orphans_purge()
    bpy.ops.outliner.orphans_purge()
    bpy.ops.outliner.orphans_purge()


def main():
    clear_scene()

    argv = sys.argv
    argv = argv[argv.index("--") + 1 :]
    fin, fout = argv

    bpy.ops.wm.usd_import(filepath=fin)
    bpy.ops.export_scene.gltf(filepath=fout, check_existing=False, export_yup=False)


if __name__ == "__main__":
    main()
