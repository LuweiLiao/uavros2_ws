from setuptools import setup


# Retain the ROS 1 package's Python package name and source layout.  The
# ament_cmake build invokes ament_python_install_package directly; this setup
# file remains usable for standard Python tooling as well.
setup(
    name='rotors_evaluation',
    version='2.2.3',
    packages=['rosbag_tools'],
    package_dir={'': 'src'},
)
