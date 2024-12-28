from setuptools import find_packages, setup

package_name = 'cf_visualizer'

data_flies = []
data_flies.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_flies.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_flies,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maximilian',
    maintainer_email='oszimilian@oszimilian.net',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'visualizer = cf_visualizer.visualizer:main'
        ],
    },
)
