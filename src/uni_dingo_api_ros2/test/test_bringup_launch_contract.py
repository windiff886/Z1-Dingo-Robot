import importlib.util
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]
PACKAGE_ROOT = REPO_ROOT / 'src' / 'uni_dingo_api_ros2'
LAUNCH_DIR = PACKAGE_ROOT / 'launch'


def _load_launch_module(path: Path):
    spec = importlib.util.spec_from_file_location(path.stem.replace('.', '_'), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class LaunchFileSmokeTest(unittest.TestCase):
    def test_core_launch_files_exist(self):
        for file_name in [
            'bringup.launch.py',
            'simulation.launch.py',
            'real.launch.py',
            'visualization.launch.py',
        ]:
            with self.subTest(file_name=file_name):
                self.assertTrue((LAUNCH_DIR / file_name).exists(), f'{file_name} must exist')

    def test_visualization_launch_is_importable(self):
        launch_path = LAUNCH_DIR / 'visualization.launch.py'
        module = _load_launch_module(launch_path)
        self.assertTrue(hasattr(module, 'generate_launch_description'))

    def test_bringup_launch_is_importable(self):
        launch_path = LAUNCH_DIR / 'bringup.launch.py'
        module = _load_launch_module(launch_path)
        self.assertTrue(hasattr(module, 'generate_launch_description'))


if __name__ == '__main__':
    unittest.main()
