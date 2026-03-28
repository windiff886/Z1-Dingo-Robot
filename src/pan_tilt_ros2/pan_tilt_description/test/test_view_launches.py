import importlib.util
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[4]
PACKAGE_ROOT = REPO_ROOT / 'src' / 'pan_tilt_ros2' / 'pan_tilt_description'
LAUNCH_DIR = PACKAGE_ROOT / 'launch'
RVIZ_DIR = PACKAGE_ROOT / 'rviz'


class PanTiltDescriptionLaunchContractTest(unittest.TestCase):
    def test_expected_view_entrypoints_exist(self):
        required_paths = [
            LAUNCH_DIR / 'view_model.launch.py',
            LAUNCH_DIR / 'view_pan_tilt_only.launch.py',
            RVIZ_DIR / 'view_model.rviz',
            RVIZ_DIR / 'pan_tilt_only.rviz',
        ]

        missing = [str(path.relative_to(REPO_ROOT)) for path in required_paths if not path.exists()]
        self.assertEqual([], missing, f'Missing expected pan-tilt view files: {missing}')

    def test_view_launches_are_importable(self):
        for file_name in ['view_model.launch.py', 'view_pan_tilt_only.launch.py']:
            with self.subTest(file_name=file_name):
                launch_path = LAUNCH_DIR / file_name
                self.assertTrue(launch_path.exists(), f'{file_name} must exist')
                spec = importlib.util.spec_from_file_location(file_name.replace('.', '_'), launch_path)
                module = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(module)
                self.assertTrue(hasattr(module, 'generate_launch_description'))


if __name__ == '__main__':
    unittest.main()
