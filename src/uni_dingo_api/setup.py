from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['uni_dingo_api', 'uni_dingo_api.api', 'uni_dingo_api.backends',
              'uni_dingo_api.backends.simulation', 'uni_dingo_api.backends.real'],
    package_dir={'': 'src'}
)

setup(**d)
