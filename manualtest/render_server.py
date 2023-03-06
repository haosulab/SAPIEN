import os

import sapien.core as sapien
import multiprocessing as mp
from multiprocessing.connection import Connection

print(f"Ray tracing disabled: {os.environ.get('SAPIEN_DISABLE_RAY_TRACING')}")


def client_fn(address, rank, conn: Connection):
    engine = sapien.Engine()
    renderer = sapien.RenderClient(address, rank)
    engine.set_renderer(renderer)
    scene = engine.create_scene()
    scene.add_ground(0)
    camera = scene.add_camera("camera_0", 128, 128, 1.0, 0.01, 10)
    camera.set_pose(sapien.Pose([0, 0, 1], [0.707, 0, 0.707, 0]))

    conn.send(True)

    while True:
        cmd, data = conn.recv()
        if cmd == "close":
            break
        elif cmd == "take_picture":
            scene._update_render_and_take_pictures([camera])
            conn.send(True)

    conn.close()


def main():
    mp.set_start_method("spawn")
    server_address = "localhost:12345"

    parent_conn, child_conn = mp.Pipe()
    p = mp.Process(target=client_fn, args=(server_address, 0, child_conn))
    p.start()

    server = sapien.RenderServer()
    server.start(server_address)

    parent_conn.recv()  # Wait for cameras to be created
    buffers = server.auto_allocate_buffers(["Color"])

    parent_conn.send(("take_picture", None))
    parent_conn.recv()
    server.wait_all()

    # print(buffers[0].shape)
    
    import numpy as np
    array = np.empty(buffers[0].shape, dtype=buffers[0].type)
    buffers[0].copy_to_host_async(array)
    buffers[0].synchronize()
    print(array.shape)

    parent_conn.send(("close", None))
    p.join()
    child_conn.close()
    parent_conn.close()


if __name__ == '__main__':
    main()


# cuda0 = torch.device("cuda:0")
# x = torch.tensor([1.0, 2.0], device=cuda0)
