import json
import os
from os.path import join as pjoin

with open('./semantic_detector.json', 'r') as f:
    big_dict = json.load(f)


def detect_motion(partnet_folder):
    assert os.path.exists(pjoin(partnet_folder, 'cues.txt'))  # mobility cue
    with open(pjoin(partnet_folder, 'meta.json'), 'r') as f:
        meta = json.load(f)
    with open(pjoin(partnet_folder, 'cues.txt'), 'r') as f:
        cues = [line.strip().split() for line in f]

    cat = meta['model_cat']
    if cat not in big_dict:
        raise f"Unknown category {cat}"
    semantics = []
    needs_check = False
    for cue in cues:
        valid_types = []
        if cue[1] in ['heavy', 'free', 'static']:
            valid_types = ['object_body']
            continue
        for name in big_dict[cat]:
            desc = big_dict[cat][name]
            if desc[0] == cue[1]:
                if any([keyword in c for c in cue[2:] for keyword in desc[1:]]):
                    valid_types.append(name)
        if len(valid_types) == 0:
            print("Cannot find valid type for: ", cue, "in category", cat, os.path.basename(partnet_folder))
            needs_check = True
        elif len(valid_types) > 1:
            print("Ambiguous part", name, desc)
            needs_check = True
        else:
            semantics.append([cue[0], valid_types[0]])
    return semantics, (os.path.basename(partnet_folder) if needs_check else 0)


DIR = '/home/fx/source/partnet-mobility-scripts/mobility_verified'
check_ids = []
for d in os.listdir(DIR):
    s, check_id = detect_motion(pjoin('/home/fx/source/partnet-mobility-scripts/mobility_verified', d))
    if check_id != 0:
        check_ids.append(check_id)
for i in check_ids:
    print(i)
