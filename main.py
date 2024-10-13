from typing import Mapping
import numpy as np
from enum import StrEnum
import itertools

# ============================= INPUTS ========================================
# Truss joints: Describes which joint is connected to which joint.
# Do not exnter reverse joint connections. For example if you have already
# entered ("A", "B") then do not enter ("B", "A") again. This is completely 
# untested and can give unexpected results.

# trusses: tuple[tuple[str, str], ...] = (
#     ("A", "B"),
#     ("A", "E"),
#     ("E", "B"),
#     ("E", "D"),
#     ("B", "D"),
#     ("B", "C"),
#     ("C", "D"),
# )

trusses: tuple[tuple[str, str], ...] = (
    ("A", "B"),
    ("A", "C"),
    ("A", "D"),
    ("B", "D"),
    ("C", "D"),
)


# Joints: Describes each joint
# Values to be filled as pre index are:
#
# 0: x-coordinate of the joint (float)
# 1: y-coordinate of the joint (float)
# 2: is there reaction force in the x direction (bool)
# 3: is there reaction force in the y direction (bool)
# 4: Point load in x-direction (float)
# 5: Point load in y-direction (float)
#
# For pin joints both 2 and 3 will be True and for roller joints either of 2 or
# 3 will be True

# joints = {
#     "A": (0, 1, 1, 1, 0, 0),
#     "B": (np.sqrt(3), 0, 0, 0, 0, 0),
#     "C": (2 * np.sqrt(3), 0, 0, 0, 0, -3),
#     "D": (np.sqrt(3), -1, 0, 0, 0, 0),
#     "E": (0, 0, 1, 0, 0, 0),
# }

joints = {
    "D": (0, 0, 1, 0, 0, 0),
    "A": (0, 3, 1, 1, 0, 0),
    "B": (4, 3, 0, 0, 0, -1),
    "C": (4, 0, 0, 0, 0, 0),
}

# ============================= END OF INPUTS =================================


def main():
    forces = unknown_forces_map()
    resolve_reactions(forces)
    next_joint = next_solvable_joint(forces)
    while next_joint != None:
        if next_joint[1] == 2:
            solve_two_unknwon_joint(next_joint[0], forces)
        else:
            solve_one_unknown_joint(next_joint[0], forces)
        next_joint = next_solvable_joint(forces)
    result_display(forces)


class Axis(StrEnum):
    X = "X"
    Y = "Y"


def resolve_reactions(forces: Mapping[str, float | None]) -> None:
    reactions_map = {}
    forces_map = {}

    for joint, joint_data in joints.items():
        if joint_data[2] == 1:
            forces[joint + "_x"] = None
            reactions_map[joint + "_x"] = joint_data[1]
        if joint_data[3] == 1:
            forces[joint + "_y"] = None
            reactions_map[joint + "_y"] = joint_data[0]

        if joint_data[4] != 0:
            forces_map[str(joint_data[4]) + "_x"] = joint_data[1]
        if joint_data[5] != 0:
            forces_map[str(joint_data[5]) + "_y"] = joint_data[0]

    A = np.zeros((len(reactions_map), len(reactions_map)), dtype=np.float32)
    b = np.zeros(len(reactions_map), dtype=np.float32)

    for i, (joint, joint_data) in enumerate(
        itertools.islice(joints.items(), len(reactions_map))
    ):

        for j, (rx_name, rx_val) in enumerate(reactions_map.items()):
            if rx_name[-1] == "x":
                index = 1
                r_axis = Axis.Y
            else:
                index = 0
                r_axis = Axis.X
            A[i, j] = cross_coff(r_axis) * (rx_val - joint_data[index])

        for force, dist in forces_map.items():
            if force[-1] == "x":
                index = 1
                r_axis = Axis.Y
            else:
                index = 0
                r_axis = Axis.X
            b[i] += (
                -cross_coff(r_axis) * float(force[0:-2]) * (dist - joint_data[index])
            )

    res = np.linalg.solve(A, b)

    for i, rxn in enumerate(reactions_map.keys()):
        forces[rxn] = res[i]


def solve_two_unknwon_joint(joint: str, forces: Mapping[str, float | None]) -> None:
    trusses = get_associated_trusses(joint)

    X = joints[joint][0]
    Y = joints[joint][1]

    A = np.zeros(shape=(2, 2), dtype=np.float32)

    b = np.array([joints[joint][4], joints[joint][5]], dtype=np.float32)

    unknown_forces = []

    for truss in trusses:
        other_end = connected_to(joint, truss)
        if forces[truss] is None:
            A[0, len(unknown_forces)] = -vec_comp(
                X, Y, joints[other_end][0], joints[other_end][1], Axis.X
            )
            A[1, len(unknown_forces)] = -vec_comp(
                X, Y, joints[other_end][0], joints[other_end][1], Axis.Y
            )
            unknown_forces.append(truss)
        else:
            b[0] += forces[truss] * vec_comp(
                X, Y, joints[other_end][0], joints[other_end][1], Axis.X
            )
            b[1] += forces[truss] * vec_comp(
                X, Y, joints[other_end][0], joints[other_end][1], Axis.Y
            )

    rxn_x = forces.get(joint + "_x")
    rxn_y = forces.get(joint + "_y")
    if rxn_x is not None:
        b[0] += rxn_x
    if rxn_y is not None:
        b[1] += rxn_y

    res = np.linalg.solve(A, b)

    forces[unknown_forces[0]] = res[0].item()
    forces[unknown_forces[1]] = res[1].item()


def solve_one_unknown_joint(joint: str, forces: Mapping[str, float | None]) -> None:
    trusses = get_associated_trusses(joint)

    X = joints[joint][0]
    Y = joints[joint][1]

    A = np.zeros(shape=(1, 1), dtype=np.float32)
    b = np.zeros(shape=1, dtype=np.float32)

    axis = None
    unknown_truss = None

    for truss in trusses:
        other_end = connected_to(joint, truss)
        if forces[truss] is None:
            unknown_truss = truss
            x_comp = vec_comp(X, Y, joints[other_end][0], joints[other_end][1], Axis.X)
            y_comp = vec_comp(X, Y, joints[other_end][0], joints[other_end][1], Axis.Y)
            if x_comp != 0:
                A[0, 0] = x_comp
                axis = Axis.X
            else:
                A[0, 0] = y_comp
                axis = Axis.Y

    for truss in trusses:
        if forces[truss] != None:
            other_end = connected_to(joint, truss)
            b[0] += forces[truss] * vec_comp(
                X, Y, joints[other_end][0], joints[other_end][1], axis
            )

    rxn_x = forces.get(joint + "_x")
    rxn_y = forces.get(joint + "_y")
    if axis == Axis.X and rxn_x != None:
        b[0] += rxn_x
    elif axis == Axis.Y and rxn_y != None:
        b[0] += rxn_y

    res = np.linalg.solve(A, b)

    forces[unknown_truss] = res[0].item()


def cross_coff(r_axis: Axis, f_axis: Axis | None = None) -> int:
    if f_axis == None:
        if r_axis == Axis.X:
            f_axis = Axis.Y
        else:
            f_axis = Axis.X

    if r_axis == Axis.X and f_axis == Axis.Y:
        return 1
    elif r_axis == Axis.Y and f_axis == Axis.X:
        return -1
    else:
        return 0


def result_display(forces: Mapping[str, float | None]):
    for k, v in forces.items():
        if v == None:
            continue
        if np.fabs(v) < 10e-7:
            v = 0
        if k[-2] == "_":
            print(k + ":", np.round(v, 3))
            continue
        print(k + ":", np.round(np.fabs(v), 3), end=" ")
        if v < 0:
            print("C")
        elif v > 0:
            print("T")
        else:
            print()


def unknown_forces_map() -> Mapping[str, float | None]:
    forces = {i[0] + i[1]: None for i in trusses}
    return forces


def connected_to(joint: str, truss: str) -> str:
    if truss[0] == joint:
        return truss[1]
    else:
        return truss[0]


def get_associated_trusses(joint: str) -> list[str]:
    res = []
    for i in trusses:
        if i[0] == joint or i[1] == joint:
            res.append(i[0] + i[1])
    return res


def unknown_force_count(joint: str, forces: Mapping[str, float | None]) -> int:
    unknowns = 0
    for truss in get_associated_trusses(joint):
        if forces[truss] == None:
            unknowns += 1

    return unknowns


def next_solvable_joint(forces: Mapping[str, float | None]) -> tuple[str, int] | None:
    for joint in joints:
        unknowns = unknown_force_count(joint, forces)
        if unknowns > 0 and unknowns < 3:
            return (joint, unknowns)
    return None


def vec_comp(jx: int, jy: int, x: int, y: int, axis: Axis) -> float:
    mod = np.sqrt((x - jx) ** 2 + (y - jy) ** 2)
    if axis == Axis.X:
        return (x - jx) / mod
    else:
        return (y - jy) / mod


if __name__ == "__main__":
    main()
