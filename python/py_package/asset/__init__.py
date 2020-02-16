import requests, zipfile, io, tempfile
import os

url = 'http://sapien.ucsd.edu/api/data/compressed/{}.zip'


def download_partnet_mobility(model_id, directory=None):
    r = requests.get(url.format(model_id), stream=True)
    if not r.ok:
        raise Exception("Download PartNet-Mobility failed")

    z = zipfile.ZipFile(io.BytesIO(r.content))

    if not directory:
        directory = 'partnet-mobility-dataset'
        os.makedirs('partnet-mobility-dataset', exist_ok=True)
    if not os.path.exists(directory):
        os.makedirs(directory)
    z.extractall(directory)
    return os.path.join(directory, str(model_id), 'mobility.urdf')
