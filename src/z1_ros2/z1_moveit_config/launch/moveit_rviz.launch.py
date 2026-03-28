from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch


def _build_moveit_config():
    return (
        MoveItConfigsBuilder('dual_z1', package_name='z1_moveit_config')
        .robot_description(file_path='config/dual_z1.urdf.xacro')
        .robot_description_semantic(file_path='config/dual_z1.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .trajectory_execution(
            file_path='config/moveit_controllers.yaml',
            moveit_manage_controllers=False,
        )
        .planning_pipelines(default_planning_pipeline='ompl', pipelines=['ompl'])
        .to_moveit_configs()
    )


def generate_launch_description():
    return generate_moveit_rviz_launch(_build_moveit_config())
