import os
from tqdm import tqdm

DIR = os.path.join('/home/fx/source/partnet-mobility-scripts/mobility_v1_alpha1')

for folder in tqdm(os.listdir(DIR)):
    os.system(f'python3 partnet.py {DIR}/{folder}')
