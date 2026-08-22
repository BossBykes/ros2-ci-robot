from setuptools import find_packages, setup

package_name = "ci_bot_monitor"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="bossbykes",
    maintainer_email="bossbykes@example.com",
    description="Runtime sensor health monitoring for CI Bot.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "sensor_health_monitor = "
            "ci_bot_monitor.sensor_health_monitor:main",
        ],
    },
)
