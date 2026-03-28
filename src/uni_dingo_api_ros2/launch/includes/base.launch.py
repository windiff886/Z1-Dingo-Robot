"""Base launch adapter for the Clearpath ROS2 base stack."""

from pathlib import Path

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource


DEFAULT_BASE_PACKAGE = 'clearpath_common'
DEFAULT_BASE_LAUNCH_FILE = 'platform.launch.py'


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[4]


def _find_vendored_package_share(package_name: str) -> Path | None:
    vendored_root = _repo_root() / 'src' / 'dingo_ros2'
    if not vendored_root.exists():
        return None

    for package_xml in vendored_root.glob('**/package.xml'):
        package_share = package_xml.parent
        if package_share.name == package_name:
            return package_share

    return None


def resolve_base_launch_artifacts(
    package_name: str = DEFAULT_BASE_PACKAGE,
    launch_file: str = DEFAULT_BASE_LAUNCH_FILE,
) -> Path:
    try:
        package_share = Path(get_package_share_directory(package_name))
    except PackageNotFoundError:
        package_share = _find_vendored_package_share(package_name)
        if package_share is None:
            raise RuntimeError(
                f"缺少底盘依赖包 '{package_name}'。"
                "请确认 src/dingo_ros2 中已包含 Clearpath ROS2 官方底盘源码，"
                "或已在工作区中构建并安装对应包。"
            ) from None

    launch_path = package_share / 'launch' / launch_file
    if not launch_path.exists():
        raise RuntimeError(
            f"底盘启动文件不存在: {launch_path}。"
            "请检查 vendored 的 Clearpath ROS2 包是否完整，"
            "并确认底盘入口仍与上游源码保持一致。"
        )

    return launch_path


def _include_external_base(_context):
    launch_path = resolve_base_launch_artifacts()

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_path)),
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=_include_external_base),
    ])
