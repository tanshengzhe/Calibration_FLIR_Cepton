from setuptools import setup

package_name = 'zed_pubsub'

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
    maintainer='pokai',
    maintainer_email='pokai@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'zed_publisher = zed_pubsub.zed_pub:main',
        'zed_subscriber = zed_pubsub.zed_sub:main',
        'frame_pub = zed_pubsub.frame_pub:main',
        'multi_topic_sub_flir = zed_pubsub.multi_topic_sub_flir:main',
        ],
    },
)
