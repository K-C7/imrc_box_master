from setuptools import find_packages, setup

package_name = 'imrc_box_master'

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
    maintainer='kei',
    maintainer_email='417keikunv2@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "box_master = imrc_box_master.box_master:main",
            "box_master_ransac = imrc_box_master.box_master_ransac:main"
        ],
    },
)
