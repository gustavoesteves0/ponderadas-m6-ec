from setuptools import find_packages, setup

package_name = 'ponderada_gustavo'

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
    maintainer='gustavoesteves0',
    maintainer_email='gme2307@gmail.com',
    description='Turtlesim',
    license='CC0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "pond_gu = ponderada_gustavo.pond_gu:main",
        ],
    },
)
