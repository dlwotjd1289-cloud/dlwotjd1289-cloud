from setuptools import find_packages, setup
package_name = "pac_robot"
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
    description="Robot capability adapters and HDP160-31 integration boundary.",
    license="Apache-2.0",
)
