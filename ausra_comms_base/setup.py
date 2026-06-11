# ============================================================
# FILE: setup.py
# PACKAGE: ausra_comms_base
# RUNS ON: Laptop (base station) only
# PURPOSE: Installs the base station package — map merge
#          launch files, config, scripts, and fake_robot_pub.
# ============================================================

import os
from glob import glob
from setuptools import setup

package_name = 'ausra_comms_base'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        # --- Package index registration ---
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # --- Package manifest ---
        ('share/' + package_name, ['package.xml']),
        # --- Launch files ---
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # --- Config files ---
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') + glob('config/*.json5')),
        # --- Shell scripts ---
        (os.path.join('share', package_name, 'scripts'),
            glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TODO',
    maintainer_email='todo@todo.com',
    description='Base station package for AUSRA swarm — map merge, diagnostics, and visualization',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'fake_robot_pub = ausra_comms_base.fake_robot_pub:main',
            'map_decompressor_node = ausra_comms_base.map_decompressor_node:main',
        ],
    },
)
