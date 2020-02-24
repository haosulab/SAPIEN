import requests
import zipfile
import io
import os

url = 'http://shapenet.org/shapenet/data/{}/{}/{}/{}/{}/{}/{}/Collada/{}.kmz'


def download_shapenet(shapenet_id, directory=None):
    ids = shapenet_id.split('_')
    if not (len(ids) == 3 and ids[0] == 'model' and ids[1] == '3dw'):
        raise Exception("Download shapenet failed: incorrect shapenet id")
    wid = ids[2]

    if not directory:
        directory = 'shapenet-dataset'
    model_dir = os.path.join(directory, shapenet_id)
    dae_dir = os.path.join(model_dir, 'models')

    if os.path.exists(dae_dir) and len(os.listdir(dae_dir)):
        return [os.path.join(dae_dir, d) for d in os.listdir(dae_dir)]

    print('downloading', shapenet_id)
    r = requests.get(url.format(wid[0], wid[1], wid[2], wid[3], wid[4], wid[5:], wid, wid), stream=True)
    if not r.ok:
        raise Exception("Download ShapeNet failed: cannot find model in database")

    z = zipfile.ZipFile(io.BytesIO(r.content))

    os.makedirs(model_dir, exist_ok=True)
    z.extractall(model_dir)

    return [os.path.join(dae_dir, d) for d in os.listdir(dae_dir)]
