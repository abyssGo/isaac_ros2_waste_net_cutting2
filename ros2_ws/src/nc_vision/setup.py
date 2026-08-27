from setuptools import find_packages, setup


package_name = "nc_vision"


setup(
    name=package_name,
    version="0.4.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="yong",
    maintainer_email="pjy12110@gmail.com",
    description="NetClean YOLO and 3D vision processing package",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "vision1_node = nc_vision.vision1_node:main",
            "vision2_node = nc_vision.vision2_node:main",
            "vision1_node_v2 = nc_vision.vision1_node_v2:main",
            "vision2_node_v2 = nc_vision.vision2_node_v2:main",
            "vision1_node_v3 = nc_vision.vision1_node_v3:main",
            "vision2_node_v3 = nc_vision.vision2_node_v3:main",
            "vision2_node_v2_3 = nc_vision.vision2_node_v2_3:main",
        ],
    },
)
