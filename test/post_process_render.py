import argparse
from tqdm import tqdm
import tifffile
import numpy as np
import os
import json
from multiprocessing import Pool

big_set = set([('Table', 'drawer'), ('Table', 'furniture_body'), ('Table', 'wheel'), ('Table', 'door'),
               ('Table', 'caster'), ('StorageFurniture', 'rotation_door'),
               ('StorageFurniture', 'furniture_body'), ('StorageFurniture', 'translation_door'),
               ('StorageFurniture', 'drawer'), ('Window', 'translation_window'), ('Window', 'window_frame'),
               ('FoldingChair', 'seat'), ('FoldingChair', 'leg'), ('Display', 'rotation_screen'),
               ('Display', 'display_base'), ('Display', 'button'),
               ('Box', 'rotation_lid'), ('Box', 'box_body'), ('CoffeeMachine', 'button'),
               ('CoffeeMachine', 'lid'), ('CoffeeMachine', 'coffee_machine_body'), ('CoffeeMachine', 'lever'),
               ('CoffeeMachine', 'knob'), ('CoffeeMachine', 'container'), ('Switch', 'switch_frame'),
               ('Switch', 'lever'), ('Switch', 'toggle_button'), ('Switch', 'slider'), ('Dispenser', 'lid'),
               ('Dispenser', 'dispenser_body'), ('Phone', 'button'), ('Phone', 'phone_base'),
               ('TrashCan', 'foot_pad'), ('TrashCan', 'lid'), ('TrashCan', 'trashcan_body'),
               ('TrashCan', 'door'), ('TrashCan', 'wheel'), ('Fan', 'rotor'), ('Fan', 'fan_frame'),
               ('Faucet', 'switch'), ('Faucet', 'faucet_base'), ('Faucet', 'spout'), ('Chair', 'wheel'),
               ('Chair', 'seat'), ('Chair', 'chair_leg'), ('Chair', 'knob'), ('Chair', 'caster'),
               ('Chair', 'lever'), ('Eyeglasses', 'leg'), ('Eyeglasses', 'glasses_body'),
               ('Dishwasher', 'rotation_door'), ('Dishwasher', 'dishwasher_body'), ('Lighter', 'wheel'),
               ('Lighter', 'button'), ('Lighter', 'lighter_body'), ('Lighter', 'rotation_lid'),
               ('Camera', 'lens'), ('Camera', 'button'), ('Camera', 'camera_body'), ('Camera', 'knob'),
               ('Knife', 'translation_blade'), ('Knife', 'knife_body'), ('Knife', 'rotation_blade'),
               ('Toilet', 'lever'), ('Toilet', 'pump_lid'), ('Toilet', 'toilet_body'), ('Toilet', 'lid'),
               ('Toilet', 'seat'), ('Toilet', 'button'), ('Scissors', 'leg'), ('Mouse', 'button'),
               ('Mouse', 'wheel'), ('Mouse', 'mouse_body'), ('Refrigerator', 'refrigerator_body'),
               ('Refrigerator', 'door'), ('USB', 'usb_rotation'), ('USB', 'usb_body'), ('USB', 'lid'),
               ('Suitcase', 'rotation_handle'), ('Suitcase', 'suitcase_body'),
               ('Suitcase', 'translation_handle'), ('Suitcase', 'wheel'), ('Suitcase', 'caster'),
               ('Door', 'door_frame'), ('Door', 'rotation_door'), ('Laptop', 'laptop_base'),
               ('Laptop', 'screen'), ('KitchenPot', 'lid'), ('KitchenPot', 'pot_body'), ('Globe', 'sphere'),
               ('Globe', 'globe_frame'), ('Bottle', 'translation_lid'), ('Bottle', 'bottle_body'),
               ('Bottle', 'rotation_lid'), ('Remote', 'button'), ('Remote', 'remote_base'), ('Pen', 'cap'),
               ('Pen', 'pen_body'), ('Pen', 'button'), ('Toaster', 'knob'), ('Toaster', 'slider'),
               ('Toaster', 'toaster_body'), ('Toaster', 'button'), ('Cart', 'wheel'), ('Cart', 'cart_body'),
               ('Kettle', 'lid'), ('Kettle', 'kettle_body'), ('Lamp', 'lamp_base'), ('Lamp', 'rotation_bar'),
               ('Lamp', 'head'), ('Safe', 'knob'), ('Safe', 'button'), ('Safe', 'safe_body'), ('Safe',
                                                                                               'door'),
               ('Clock', 'hand'), ('Clock', 'clock_body'), ('Pliers', 'leg'), ('Printer', 'button'),
               ('Printer', 'printer_body'), ('Bucket', 'handle'), ('Bucket', 'bucket_body'),
               ('Keyboard', 'keyboard_base'), ('Keyboard', 'key'), ('Stapler', 'stapler_body'),
               ('Stapler', 'lid'), ('Stapler', 'stapler_base'), ('WashingMachine', 'door'),
               ('WashingMachine', 'knob'), ('WashingMachine', 'button'),
               ('WashingMachine', 'washing_machine_body'), ('Oven', 'door'), ('Oven', 'knob'),
               ('Oven', 'oven_body'), ('Oven', 'translation_tray'), ('Oven', 'button'), ('Microwave', 'door'),
               ('Microwave', 'microwave_body'), ('Microwave', 'button')])


def process_by_config(configfile):
    with open(os.path.join(configfile), 'r') as f:
        config = json.load(f)
    D = os.path.dirname(configfile)
    L = os.path.basename(configfile).split('_')[:2]
    segfile = os.path.join(D, '_'.join(L + ['segmentation.tif']))
    pcfile = os.path.join(D, '_'.join(L + ['pointcloud.npz']))

    def find_id(link):
        target = (config['category'], link['semantic'])
        if target in big_set:
            return link['id']
        if link['parent'] not in config['links']:
            return -1
        return find_id(config['links'][link['parent']])

    for link in config['links']:
        config['links'][link]['change_id_to'] = find_id(config['links'][link])

    change_id_dict = {}
    for name, link in config['links'].items():
        change_id_dict[link['id']] = link['change_id_to']

    change_id_array = np.zeros(max(change_id_dict.keys()) + 1)
    for k, v in change_id_dict.items():
        change_id_array[k] = v

    seg = tifffile.imread(segfile)
    seg2 = change_id_array[seg]
    tifffile.imwrite(f'{segfile[:-4]}2.tif', seg2, compress=6, photometric='minisblack')

    pc = np.load(pcfile)['pc']
    pc[:, :, -1] = seg2
    np.savez_compressed(f'{pcfile[:-4]}2', pc=pc)


parser = argparse.ArgumentParser()
parser.add_argument('--imagedir', type=str, required=True)
args = parser.parse_args()
imagedir = args.imagedir

configfiles = [f for f in os.listdir(imagedir) if f.endswith('config.json')]
configfiles = [os.path.join(imagedir, f) for f in configfiles]

with Pool() as p:
    r = list(tqdm(p.imap(process_by_config, configfiles), total=len(configfiles)))

