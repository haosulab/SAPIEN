import os
import re

PARTNET_DIR = '/home/sim/mobility_dataset/mobility_v1_alpha5'
FILENAME = 'cues.txt'


def main():
    shape_dirs = os.listdir(PARTNET_DIR)
    joint_key = "slider"
    part_key = ["drawer", "handle"]
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
    print(id_list)
    print(len(id_list))


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


if __name__ == '__main__':
    main()
