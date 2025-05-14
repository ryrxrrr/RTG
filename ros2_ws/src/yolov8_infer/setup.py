from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'yolov8_infer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # 让ROS2在执行时能找到该包
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        # 安装 package.xml
        ('share/' + package_name, ['package.xml']),

        # 让ROS2能够找到 msg 文件(如果你有多份.msg，都能被glob匹配)
        (os.path.join('share', package_name), glob('msg/*.msg')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='royin',
    maintainer_email='royin@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 原有的节点入口，比如：
            'yolo_infer = yolov8_infer.yolo_infer_node:main',
        ],
    },
)
