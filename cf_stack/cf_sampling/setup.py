from setuptools import find_packages, setup

package_name = 'cf_sampling'

data_flies = []
data_flies.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_flies.append(('share/' + package_name, ['package.xml']))
data_flies.append(('share/' + package_name + '/launch', ['launch/cf_sampling.launch.py']))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_flies,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rosrunner',
    maintainer_email='you@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sampling = cf_sampling.sampling:main'
        ],
    },
)
