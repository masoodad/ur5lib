from setuptools import setup, find_packages

setup(
    name='ur5lib',
    version='0.1.0',
    author='Masood Ahmad',
    description='Modular Python library for controlling a UR5 robot (real/sim)',
    packages=find_packages(),
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
