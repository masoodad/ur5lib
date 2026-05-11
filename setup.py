from setuptools import setup, find_packages

_subpackages = [p for p in find_packages() if p not in ('examples', 'testlib')]

setup(
    name='ur5lib',
    version='1.0.1',
    author='Masood Ahmad',
    description='Modular Python library for controlling a UR5 robot (real/sim)',
    packages=['ur5lib'] + ['ur5lib.' + p for p in _subpackages],
    package_dir={'ur5lib': '.'},
    install_requires=[
        'numpy',
        'ur_rtde',   # optional: if using RTDE
    ],
    entry_points={
        'console_scripts': [
            'ur5lib-cli = ur5lib.cli:main'
        ],
    },
    python_requires='>=3.7',
)
