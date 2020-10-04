import requests
import zipfile
import io
import os

url = 'https://sapien.ucsd.edu/api/download/compressed/{}.zip?token={}'


def download_partnet_mobility(model_id, token=None, directory=None):
    if token is None:
        if 'SAPIEN_ACCESS_TOKEN' not in os.environ:
            raise Exception('To download the model, you need to provide the token or set the SAPIEN_ACCESS_TOKEN environment variable.')
        token = os.environ['SAPIEN_ACCESS_TOKEN']

    if not directory:
        directory = 'partnet-mobility-dataset'
    urdf_file = os.path.join(directory, str(model_id), 'mobility.urdf')

    # return if file exists
    if os.path.exists(urdf_file):
        return urdf_file

    # download file
    print('downloading from', url.format(model_id, token))
    r = requests.get(url.format(model_id, token), stream=True)
    if not r.ok:
        raise Exception("Download PartNet-Mobility failed. "
                        "Please check your token and IP address."
                        "Also make sure sure the model id is valid")

    z = zipfile.ZipFile(io.BytesIO(r.content))

    os.makedirs(directory, exist_ok=True)
    z.extractall(directory)
    return urdf_file
