 .. _inverse_kinematics:

Inverse Kinematics
==================

Inverse kinematics determine the joint positions that provide the desired pose for the robot's end-effectors. In ``mplib``, you can solve the inverse kinematics of the ``move_group`` link with:

.. code:: python

    planner.IK(target_pose, init_qpos, n_init_qpos = 20, threshold = 1e-3)

``Planner.IK()`` internally implements a numerical method and takes the following arguments:

- ``target_pose``: a 7-dim list specifies the target pose of the ``move_group`` link. The first three elements describe the position part, and the remaining four elements describe the quaternion (wxyz) for the rotation part.
- ``init_qpos``: a list describes the joint positions of all the active joints (e.g., given by SAPIEN). It will be used as the initial state for the numerical method.
- ``n_init_qpos=20``: besides the provided initial state, the method also samples extra initial states to run the algorithm for at most ``n_init_qpos`` times. In this way, it can avoid local minimums and increase the success rate.
- ``threshold=1e-3``: a threshold for determining whether the calculated pose is close enough to to the target pose.
 
It returns a tuple of two elements:

- ``status``: a string indicates the status.
- ``result``: a NumPy array describes the calculated joint positions.

If ``planner.IK()`` fails, please increase ``n_init_qpos`` or double-check whether the target pose is reachable.

