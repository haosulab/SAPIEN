import os
import re
import shutil
import glob

PARTNET_DIR = '/home/sim/project/mobility-v0-prealpha3/mobility_verified'
MINGHUA_DIR = '/home/sim/project/storage_furniture'
FILENAME = 'cues.txt'


def get_partnet_id():
    shape_dirs = os.listdir(PARTNET_DIR)
    joint_key = "hinge"
    part_key = ["door", "handle"]
    meta = "StorageFurniture"
    id_list = []
    for shape_dir in shape_dirs:
        partnet_id = shape_dir
        shape_dir = os.path.join(PARTNET_DIR, shape_dir)
        if meta:
            if not find_meta(shape_dir, meta):
                continue
        filepath = os.path.join(shape_dir, FILENAME)
        if find_key_words(filepath, joint_key, part_key):
            id_list.append(partnet_id)

    id_list = sorted(id_list)
    print(len(id_list))
    return id_list


def find_key_words(path, joint, semantics):
    with open(path, 'r') as f:
        lines = f.readlines()
        for line in lines:
            if not re.search(joint, line):
                pass
            else:
                found = True
                for semantic in semantics:
                    found = found and re.search(semantic, line)
                if found:
                    return True
    return False


def find_meta(path, meta):
    with open(os.path.join(path, "meta.json")) as f:
        lines = f.readlines()
        found = False
        for line in lines:
            if re.search(meta, line):
                return True
        return found


def merge_with_minghua(output_dir, ids):
    if not os.path.exists(output_dir):
        os.mkdir(output_dir)
    obj_folder = os.path.join(MINGHUA_DIR, "new_objs")
    urdf_folder = os.path.join(MINGHUA_DIR, "urdfs")

    for id in ids:
        input_folder = os.path.join(PARTNET_DIR, id)
        output_folder = os.path.join(output_dir, id)
        output_obj_folder = os.path.join(output_folder, "new_objs")
        assert os.path.exists(input_folder)
        assert not os.path.exists(output_folder)
        shutil.copytree(input_folder, output_folder)
        os.remove(os.path.join(output_folder, "mobility.urdf"))
        shutil.copyfile(os.path.join(urdf_folder, "{}.urdf".format(id)), os.path.join(output_folder, "mobility.urdf"))

        # Copy all obj
        link_objs = glob.glob("{}/{}_link_*/*".format(obj_folder, id))
        os.mkdir(output_obj_folder)
        for link_objs in link_objs:
            shutil.copy2(link_objs, output_obj_folder)


if __name__ == '__main__':
    ids = get_partnet_id()
    merge_with_minghua("/home/sim/project/mobility_convex", ids)
