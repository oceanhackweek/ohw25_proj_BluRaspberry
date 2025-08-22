from setuptools import find_packages, setup
import os

package_name = 'ocean_score_wrapper'

virtualenv_name = ".env"
home_path = os.path.expanduser("~")
executable_path = os.path.join(home_path, 'src/etc/ohw25_proj_BluRaspberry/contributor_folders/ELine', virtualenv_name, 'bin', 'python')

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='erin',
    maintainer_email='erin@robotics88.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'score_node = ocean_score_wrapper.score_node:main',
        ],
    },
    options={
        'build_scripts': {
            'executable': executable_path,
        }
    }
)
