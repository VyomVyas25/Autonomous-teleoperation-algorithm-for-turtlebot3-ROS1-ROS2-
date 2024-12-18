from setuptools import find_packages, setup

package_name = 'py_pubsub'

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
    maintainer='vyom',
    maintainer_email='vyom@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = py_pubsub.publisher_member_function:main',
            'listener = py_pubsub.subscriber_member_function:main',
            'camNode = py_pubsub.cam:main',
            'ttlsim = py_pubsub.ttlsim:main',
            'spiral = py_pubsub.fibo:main',
            'fib = py_pubsub.fib:main',
            'cam = py_pubsub.chodu:main',
            'feed = py_pubsub.feed:main',



        ],
    },
)
