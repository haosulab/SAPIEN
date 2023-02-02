from __future__ import annotations
import sapien.core.pysapien.coacd
import typing
import sapien.core.pysapien

__all__ = [
    "run_coacd",
    "set_log_level"
]


def run_coacd(mesh: sapien.core.pysapien.NonconvexMeshGeometry, threshold: float = 0.05, preprocess: bool = True, preprocess_resolution: int = 30, pca: bool = False, merge: bool = True, mcts_max_depth: int = 3, mcts_nodes: int = 20, mcts_iterations: int = 150, seed: int = 0) -> typing.List[sapien.core.pysapien.ConvexMeshGeometry]:
    pass
def set_log_level(level: str) -> None:
    pass
