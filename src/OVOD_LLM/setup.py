from setuptools import setup

package_name = 'ovod_llm'

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
    description='Webcam Publisher node / YOLOWorld Detection Node / Text Query Publisher Node / LLM VQA Node',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            
            'webcam_publisher = ovod_llm.camera_node:main',

            'detection_node_YW = ovod_llm.detection_node_YW:main',
            'detection_node_YW_query = ovod_llm.detection_node_YW_query:main',
            'detection_node_YW_query_clip = ovod_llm.detection_node_YW_query_clip:main',
            
            # 'hierarchy_detection_node = ovod_llm.hierarchy_detection_node:main',
            # 'detection_node_gd = ovod_llm.detection_node_gd:main',
            'user_input_query_node = ovod_llm.user_input_query_node:main',
            'llm_node = ovod_llm.llm_node:main',
        ],
    },
)
