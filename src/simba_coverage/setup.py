from setuptools import find_packages, setup

package_name = 'simba_coverage'
setup(
    name=package_name,
    version='0.3.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='philip',
    maintainer_email='philip@example.com',
    description='Fixed-map indoor coverage planning, supervision, and measurement.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'coverage_supervisor = simba_coverage.supervisor:main',
        'coverage_meter = simba_coverage.meter:main',
        'coverage_preview = simba_coverage.preview:preview_main',
    ]},
)
