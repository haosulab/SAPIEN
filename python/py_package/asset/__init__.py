import requests
import zipfile
import io
import os

url = 'https://sapien.ucsd.edu/api/download/compressed/{}.zip?token={}'


def download_partnet_mobility(model_id, token, directory=None):
    if not directory:
        directory = 'partnet-mobility-dataset'
    urdf_file = os.path.join(directory, str(model_id), 'mobility.urdf')

    # return if file exists
    if os.path.exists(urdf_file):
        return urdf_file

    # download file
    r = requests.get(url.format(model_id, token), stream=True)
    if not r.ok:
        raise Exception("Download PartNet-Mobility failed")

    z = zipfile.ZipFile(io.BytesIO(r.content))

    os.makedirs(directory, exist_ok=True)
    z.extractall(directory)
    return urdf_file
