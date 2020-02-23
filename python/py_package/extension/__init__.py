import requests
import zipfile
import io
import os

url = 'http://shapenet.org/shapenet/data/{}/{}/{}/{}/{}/{}/{}/Collada/{}.kmz'


def download_shapenet(shapenet_id, directory=None):
    ids = shapenet_id.split('_')
    if not (len(ids) == 3 and ids[0] == '3dw' and ids[1] == '3dw'):
        raise Exception("Download shapenet failed: incorrect shapenet id")
    wid = shapenet_id[2]

    if not directory:
        directory = 'shapenet-dataset'
    model_dir = os.path.join(directory, shapenet_id)
    dae_file = os.path.join(model_dir, 'models/model.dae')

    if os.path.exists(dae_file):
        return dae_file

    print('downloading', shapenet_id)
    r = requests.get(url.format(wid[0], wid[1], wid[2], wid[3], wid[4], wid[5:], wid, wid), stream=True)
    if not r.ok:
        raise Exception("Download ShapeNet failed: cannot find model in database")

    z = zipfile.ZipFile(io.BytesIO(r.content))

    os.makedirs(model_dir, exist_ok=True)
    z.extractall(model_dir)

    return dae_file
