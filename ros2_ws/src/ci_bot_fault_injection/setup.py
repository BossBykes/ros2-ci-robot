from setuptools import find_packages, setup

package_name = "ci_bot_fault_injection"

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
    description="Controlled sensor fault injection for CI Bot regression testing.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "scan_fault_injector = "
            "ci_bot_fault_injection.scan_fault_injector:main",
        ],
    },
)
