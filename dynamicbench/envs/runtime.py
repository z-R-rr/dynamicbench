"""Version boundaries; public DynamicBench poses remain WXYZ on both runtimes."""

from importlib.metadata import PackageNotFoundError, version


def sim_version():
    try:
        return version("isaacsim")
    except PackageNotFoundError:
        return "unavailable"


ISAAC61 = sim_version().startswith("6.1.")


def tensor(value):
    return value.torch if ISAAC61 else value


def lab_quaternion(wxyz):
    return tuple(wxyz[1:]) + (wxyz[0],) if ISAAC61 else tuple(wxyz)


def public_pose(value):
    result = tensor(value).numpy().copy()
    if ISAAC61:
        result[..., 3:7] = result[..., [6, 3, 4, 5]]
    return result


def joint_state(robot, position, velocity):
    if ISAAC61:
        robot.write_joint_position_to_sim_index(position=position)
        robot.write_joint_velocity_to_sim_index(velocity=velocity)
    else:
        robot.write_joint_state_to_sim(position, velocity)


def position_target(robot, target):
    if ISAAC61:
        robot.set_joint_position_target_index(target=target)
    else:
        robot.set_joint_position_target(target)


def velocity_target(robot, target):
    if ISAAC61:
        robot.set_joint_velocity_target_index(target=target)
    else:
        robot.set_joint_velocity_target(target)


def root_state(obj, state):
    if ISAAC61:
        obj.write_root_pose_to_sim_index(root_pose=state[:, :7].contiguous())
        obj.write_root_velocity_to_sim_index(root_velocity=state[:, 7:].contiguous())
    else:
        obj.write_root_state_to_sim(state)


def runtime_metadata():
    result = {"isaacsim": sim_version(), "physics_backend": "PhysX", "physics_device": "cpu"}
    for name in ("isaaclab", "isaaclab-physx", "torch", "warp-lang"):
        try:
            result[name] = version(name)
        except PackageNotFoundError:
            pass
    result["lab_quaternion_order"] = "xyzw" if ISAAC61 else "wxyz"
    result["dataset_quaternion_order"] = "wxyz"
    return result
