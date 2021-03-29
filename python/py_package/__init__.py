try:
    import torch  # required if sapien is compiled with render-to-torch-tensor
except ImportError:
    pass
from sapien import core, asset, example, utils
