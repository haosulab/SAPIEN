from xml.etree import ElementTree as ET
from ..pysapien.physx import PhysxArticulationLinkComponent, PhysxArticulationJoint


def export_link(link: PhysxArticulationLinkComponent):
    cmass_pose = link.cmass_local_pose
    angles = cmass_pose.rpy

    elem_link = ET.Element("link", {"name": f"link_{link.index}"})
    elem_inertial = ET.SubElement(elem_link, "inertial")
    ET.SubElement(
        elem_inertial,
        "origin",
        {
            "xyz": f"{cmass_pose.p[0]} {cmass_pose.p[1]} {cmass_pose.p[2]}",
            "rpy": f"{angles[0]} {angles[1]} {angles[2]}",
        },
    )
    ET.SubElement(elem_inertial, "mass", {"value": str(link.mass)})
    ET.SubElement(
        elem_inertial,
        "inertia",
        {
            "ixx": str(link.inertia[0]),
            "iyy": str(link.inertia[1]),
            "izz": str(link.inertia[2]),
            "ixy": "0",
            "ixz": "0",
            "iyz": "0",
        },
    )

    return elem_link


def export_joint(joint: PhysxArticulationJoint):
    name = f"joint_{joint.child_link.index}"
    if joint.parent_link is None:
        type = "fixed" if joint.type == "fixed" else "floating"
        elem_joint = ET.Element("joint", {"name": name, "type": type})
        ET.SubElement(elem_joint, "parent", {"link": "world"})
        ET.SubElement(elem_joint, "child", {"link": f"link_{joint.child_link.index}"})
        return [elem_joint]

    if joint.type == "fixed":
        type = "fixed"
    elif joint.type == "prismatic":
        type = "prismatic"
    elif joint.type.startswith("revolute"):
        if joint.limit[0][0] < -1e5 and joint.limit[0][1] > 1e5:
            type = "continuous"
        else:
            type = "revolute"
    else:
        raise Exception(f"invalid joint type {joint.type}")

    j2p = joint.pose_in_parent
    c2j = joint.pose_in_child.inv()

    angles = j2p.rpy

    # dummy link at joint frame
    elem_dummy_link = ET.Element(
        "link", {"name": f"link_dummy_{joint.child_link.index}"}
    )

    # joint connecting parent and dummy
    elem_joint = ET.Element("joint", {"name": name, "type": type})
    ET.SubElement(
        elem_joint,
        "origin",
        {
            "xyz": f"{j2p.p[0]} {j2p.p[1]} {j2p.p[2]}",
            "rpy": f"{angles[0]} {angles[1]} {angles[2]}",
        },
    )
    ET.SubElement(elem_joint, "axis", {"xyz": "1 0 0"})
    ET.SubElement(elem_joint, "parent", {"link": f"link_{joint.parent_link.index}"})
    ET.SubElement(elem_joint, "child", {"link": f"link_dummy_{joint.child_link.index}"})
    if type == "prismatic" or type.startswith("revolute"):
        ET.SubElement(
            elem_joint,
            "limit",
            {
                "effort": "0",
                "velocity": "0",
                "lower": str(max(joint.limit[0][0], -1e5)),
                "upper": str(min(joint.limit[0][1], 1e5)),
            },
        )

    # fixed joint connecting dummy and child
    elem_dummy_joint = ET.Element(
        "joint", {"name": f"joint_dummy_{joint.child_link.index}", "type": "fixed"}
    )
    angles = c2j.rpy

    ET.SubElement(
        elem_dummy_joint,
        "origin",
        {
            "xyz": f"{c2j.p[0]} {c2j.p[1]} {c2j.p[2]}",
            "rpy": f"{angles[0]} {angles[1]} {angles[2]}",
        },
    )
    ET.SubElement(elem_dummy_joint, "axis", {"xyz": "0 0 0"})
    ET.SubElement(
        elem_dummy_joint, "parent", {"link": f"link_dummy_{joint.child_link.index}"}
    )
    ET.SubElement(elem_dummy_joint, "child", {"link": f"link_{joint.child_link.index}"})

    return [elem_joint, elem_dummy_link, elem_dummy_joint]


def export_kinematic_chain_xml(articulation):
    elem_robot = ET.Element("robot", {"name": ""})
    ET.SubElement(elem_robot, "link", {"name": "world"})

    for l in articulation.links:
        elem_robot.append(export_link(l))

    for j in articulation.joints:
        for e in export_joint(j):
            elem_robot.append(e)

    return elem_robot


def export_kinematic_chain_urdf(articulation, force_fix_root=False):
    xml = export_kinematic_chain_xml(articulation)
    if force_fix_root:
        for j in xml.findall("joint"):
            if j.attrib["type"] == "floating":
                j.attrib["type"] = "fixed"

    return ET.tostring(xml, encoding="utf8").decode()
