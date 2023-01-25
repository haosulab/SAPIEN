import requests
import zipfile
import io
import os
from pathlib import Path
import numpy as np
import sapien
from typing import Tuple

url = "https://sapien.ucsd.edu/api/download/compressed/{}.zip?token={}"


def download_partnet_mobility(model_id, token=None, directory=None):
    if not directory:
        directory = os.environ.get("PARTNET_MOBILITY_DATASET")
        if not directory:
            directory = "partnet-mobility-dataset"
    urdf_file = os.path.join(directory, str(model_id), "mobility.urdf")

    # return if file exists
    if os.path.exists(urdf_file):
        return urdf_file

    if token is None:
        if "SAPIEN_ACCESS_TOKEN" not in os.environ:
            raise Exception(
                "To download the model, you need to provide the token or set the SAPIEN_ACCESS_TOKEN environment variable."
            )
        token = os.environ["SAPIEN_ACCESS_TOKEN"]

    # download file
    print("downloading from", url.format(model_id, token))
    r = requests.get(url.format(model_id, token), stream=True)
    if not r.ok:
        raise Exception(
            "Download PartNet-Mobility failed. "
            "Please check your token and IP address."
            "Also make sure sure the model id is valid"
        )

    z = zipfile.ZipFile(io.BytesIO(r.content))

    os.makedirs(directory, exist_ok=True)
    z.extractall(directory)
    return urdf_file


def create_dome_envmap(
    filename="/tmp/sapien_dome.ktx",
    sky_color=[0.6, 0.6, 0.6],
    ground_color=[0.2, 0.2, 0.2],
    blend=0.3,
    resolution=256,
):
    assert str(filename).endswith(".ktx"), "generated envmap must end with .ktx"
    path = Path(filename)
    path.parent.mkdir(exist_ok=True)
    from sapien.core import SapienRenderer
    import numpy as np
    from PIL import Image
    import tempfile

    sky_color = np.array(sky_color)
    ground_color = np.array(ground_color)

    py = (np.ones((resolution, resolution, 3)) * sky_color * 255).astype(np.uint8)
    ny = (np.ones((resolution, resolution, 3)) * ground_color * 255).astype(np.uint8)
    other = np.zeros((resolution, resolution, 3), dtype=np.uint8)

    c = np.linspace(0.5, resolution - 0.5, resolution) * 2 / resolution - 1
    coords = np.stack(np.meshgrid(c, c), 0)
    angles = np.arctan2(coords[1], np.sqrt(coords[0] ** 2 + 1))
    r = (angles / blend).clip(-1, 1) * 0.5 + 0.5
    other = (1 - r[..., None]) * sky_color + r[..., None] * ground_color
    other = (other.clip(0, 1) * 255).astype(np.uint8)

    with tempfile.TemporaryDirectory() as d:
        fpy = os.path.join(d, "py.png")
        fny = os.path.join(d, "ny.png")
        fother = os.path.join(d, "other.png")

        Image.fromarray(py).save(fpy)
        Image.fromarray(ny).save(fny)
        Image.fromarray(other).save(fother)

        renderer = SapienRenderer()
        renderer.create_ktx_environment_map(
            fother, fother, fpy, fny, fother, fother, filename
        )
    return filename


def create_checkerboard(
    renderer: sapien.core.IPxrRenderer,
    shape: tuple,
    length=0.1,
    color1=[1, 1, 1, 1],
    color2=[0, 0, 0, 1],
) -> Tuple[sapien.core.RenderMesh, sapien.core.RenderMaterial]:
    array = np.zeros([*shape, 4])
    checker = np.indices(shape).sum(axis=0) % 2
    array[checker == 0] = color1
    array[checker == 1] = color2

    array = (array * 255).clip(0, 255).astype(np.uint8)

    texture = renderer.create_texture_from_array(array, filter_mode="nearest")
    mat = renderer.create_material()
    mat.set_diffuse_texture(texture)
    mat.set_base_color([1, 0, 1, 1])

    width = length * shape[0]
    height = length * shape[1]
    mesh = renderer.create_mesh(
        np.array(
            [
                [0, 0, 0],
                [width, 0, 0],
                [width, height, 0],
                [0, height, 0],
            ],
            dtype=np.float32,
        ),
        np.array([[0, 1, 2], [0, 2, 3]], dtype=np.uint32),
    )
    mesh.set_uvs(np.array([[0, 0], [0, 1], [1, 1], [1, 0]], dtype=np.float32))
    mesh.set_normals(
        np.array([[0, 0, 1], [0, 0, 1], [0, 0, 1], [0, 0, 1]], dtype=np.float32)
    )

    return mesh, mat
