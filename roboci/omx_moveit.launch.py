"""OpenMANIPULATOR-X + MoveIt 2 in Gazebo, headless, for RoboCI.

Fork file (roboci branch): it replicates open_manipulator_bringup's
open_manipulator_x_gazebo.launch.py composition (the same xacro mappings,
spawners and clock bridge) plus the vendor MoveIt launch's move_group,
because the vendor launches take no description override (the wrong_tcp
control and the pick_and_place DetachableJoint both need one), no
joint_states remap (joint_state_noise needs the stack to read the
relayed stream), and no headless flag. Two deliberate fixes over the
vendor files: robot_state_publisher gets use_sim_time (the vendor gazebo
launch omits it, so TF would carry wall time while the sim runs slower),
and move_group always runs with use_sim_time and no RViz.

Arguments:
  world         the world file (robot-tests.yaml passes ${world}); the
                vendor world NAME is derived from its basename.
  gui           "true" starts the Gazebo GUI like the vendor launch does
                (default "false": gz -s, server only).
  box_in_scene  "true" mirrors the target box into MoveIt's planning
                scene (roboci_scene.py); "false" is the missing_box
                control: the box stays in Gazebo, MoveIt plans as if it
                weren't there.
  box_center / box_size  the target box geometry (must match the
                scenario's spawn_object).
  tcp_offset    meters added to the end_effector_joint origin in
                MOVEIT's copy of the URDF only (default 0). 0.03 is the
                wrong_tcp control: MoveIt reports success while the real
                tip ends up 3 cm from where MoveIt thinks it is.
  attach_object the spawned model a DetachableJoint holds (grasp
                decision D1, docs/manipulation-plan.md; default "": none).
                RoboCI commands it from its gripper steps and labels the
                run physics-assisted.
"""

import os
from pathlib import Path
from xml.dom import minidom

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

HERE = os.path.dirname(os.path.abspath(__file__))

# The engine's joint_state relay (robot.sensors "joints"): the stack's
# view of the joints, noised by the sensor_noise fault. robot_state_publisher
# and move_group read it; the broadcaster still publishes /joint_states for
# the engine's readiness watch and ground truth.
JOINTS_RELAY = "/roboci/sensors/joints"


def _flag(context, name):
    return LaunchConfiguration(name).perform(context).strip().lower() in ("true", "1", "yes")


def _world_name(context):
    path = LaunchConfiguration("world").perform(context)
    base = os.path.basename(path)
    return base[:-4] if base.endswith(".sdf") else base


def _xacro_file():
    return os.path.join(
        get_package_share_directory("open_manipulator_description"),
        "urdf",
        "open_manipulator_x",
        "open_manipulator_x.urdf.xacro",
    )


def _sim_description(attach_object):
    """The simulator's URDF, processed exactly as the vendor gazebo
    launch processes it, plus the DetachableJoint for attach_object."""
    doc = xacro.process_file(_xacro_file(), mappings={"use_sim": "true"})
    if attach_object:
        t = {
            "attach": f"/roboci/attach/{attach_object}",
            "detach": f"/roboci/detach/{attach_object}",
            "state": f"/roboci/attach_state/{attach_object}",
        }
        frag = minidom.parseString(
            "<robot><gazebo>"
            '<plugin filename="gz-sim-detachable-joint-system"'
            ' name="gz::sim::systems::DetachableJoint">'
            "<parent_link>link5</parent_link>"
            f"<child_model>{attach_object}</child_model><child_link>link</child_link>"
            f"<attach_topic>{t['attach']}</attach_topic>"
            f"<detach_topic>{t['detach']}</detach_topic>"
            f"<output_topic>{t['state']}</output_topic>"
            "<suppress_child_warning>true</suppress_child_warning>"
            "</plugin></gazebo></robot>"
        ).documentElement
        for node in list(frag.childNodes):
            doc.documentElement.appendChild(doc.importNode(node, True))
    return doc.toprettyxml(indent="  ")


def _moveit_description(tcp_offset):
    """MoveIt's URDF: the vendor MoveIt launch builds it with the xacro
    defaults (use_sim false), so this does the same, then shifts the tool
    frame for the wrong_tcp control."""
    doc = xacro.process_file(_xacro_file(), mappings={"use_sim": "false"})
    if tcp_offset:
        joint = next(
            (j for j in doc.getElementsByTagName("joint")
             if j.getAttribute("name") == "end_effector_joint"),
            None,
        )
        if joint is None:
            raise RuntimeError("end_effector_joint not found in the X description")
        origin = joint.getElementsByTagName("origin")[0]
        xyz = [float(v) for v in (origin.getAttribute("xyz") or "0 0 0").split()]
        xyz[0] += tcp_offset
        origin.setAttribute("xyz", " ".join(repr(v) for v in xyz))
    return doc.toprettyxml(indent="  ")


def _bringup(context):
    bringup_path = get_package_share_directory("open_manipulator_bringup")
    desc_path = get_package_share_directory("open_manipulator_description")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch"),
            "/gz_sim.launch.py",
        ]),
        launch_arguments=[
            ("gz_args", f"{_world_name(context)}.sdf -v 1 -r"
             + ("" if _flag(context, "gui") else " -s")),
        ],
    )

    robot_desc = _sim_description(
        LaunchConfiguration("attach_object").perform(context).strip())

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_desc},
            {"use_sim_time": True},
        ],
        remappings=[("/joint_states", JOINTS_RELAY)],
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string", robot_desc,
            "-x", "0.0", "-y", "0.0", "-z", "0.0",
            "-R", "0.0", "-P", "0.0", "-Y", "0.0",
            "-name", "open_manipulator_x",
            "-allow_renaming", "true",
            "-use_sim", "true",
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
        output="screen",
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller"],
        output="screen",
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    return [
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[arm_controller_spawner, gripper_controller_spawner],
            )
        ),
        SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=[
                os.path.join(bringup_path, "worlds"),
                ":" + str(Path(desc_path).parent.resolve()),
            ],
        ),
        bridge,
        gazebo,
        node_robot_state_publisher,
        gz_spawn_entity,
    ]


def _move_group(context):
    from moveit_configs_utils import MoveItConfigsBuilder

    tcp_offset = float(LaunchConfiguration("tcp_offset").perform(context) or 0.0)
    # The vendor open_manipulator_x_moveit.launch.py builder chain, without RViz.
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="open_manipulator_x", package_name="open_manipulator_moveit_config")
        .robot_description_semantic(
            str(Path("config") / "open_manipulator_x" / "open_manipulator_x.srdf"))
        .joint_limits(str(Path("config") / "open_manipulator_x" / "joint_limits.yaml"))
        .trajectory_execution(
            str(Path("config") / "open_manipulator_x" / "moveit_controllers.yaml"))
        .robot_description_kinematics(
            str(Path("config") / "open_manipulator_x" / "kinematics.yaml"))
        .to_moveit_configs()
    )
    params = moveit_config.to_dict()
    params["robot_description"] = _moveit_description(tcp_offset)
    return [
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                params,
                {
                    "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
                    "warehouse_host": os.path.expanduser("~/.ros/warehouse_ros.sqlite"),
                },
                {"use_sim_time": True, "publish_robot_description_semantic": True},
            ],
            remappings=[("/joint_states", JOINTS_RELAY)],
        )
    ]


def _scene(context):
    cmd = [
        "python3", os.path.join(HERE, "roboci_scene.py"),
        "--frame", "link1",
        "--box-center", LaunchConfiguration("box_center").perform(context),
        "--box-size", LaunchConfiguration("box_size").perform(context),
    ]
    if not _flag(context, "box_in_scene"):
        cmd.append("--no-box")
    return [ExecuteProcess(cmd=cmd, output="screen")]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="empty_world.sdf"),
        DeclareLaunchArgument("gui", default_value="false"),
        DeclareLaunchArgument("box_in_scene", default_value="true"),
        DeclareLaunchArgument("box_center", default_value="0.20 -0.15 0.05"),
        DeclareLaunchArgument("box_size", default_value="0.10 0.10 0.10"),
        DeclareLaunchArgument("tcp_offset", default_value="0.0"),
        DeclareLaunchArgument("attach_object", default_value=""),
        OpaqueFunction(function=_bringup),
        OpaqueFunction(function=_move_group),
        OpaqueFunction(function=_scene),
    ])
