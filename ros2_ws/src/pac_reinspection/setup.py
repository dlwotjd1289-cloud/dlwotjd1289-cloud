from setuptools import find_packages, setup
package_name = "pac_reinspection"
setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="PAC 2026 Team",
    description="Stage 2 reinspection (RI) reasons on top of pac_runtime StateValidator.",
    license="Apache-2.0",
)
