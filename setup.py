from setuptools import setup

package_name = 'delivery'

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
    maintainer='soar',
    maintainer_email='joony414@g.skku.edu',
    description='Delivery node for SOAR Drone',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'delivery_node = delivery.delivery_node:main'
        ],
    },
)
