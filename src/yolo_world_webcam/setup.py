from setuptools import setup

package_name = 'yolo_world_webcam'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jclee',
    maintainer_email='jclee@rcv.sejong.ac.kr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	"yolo_world = yolo_world_webcam.yolo_world_webcam_subscriber:main",
        ],
    },
)
