from setuptools import find_packages, setup


package_name = "nc_vision"
model_file = "../../../models/netclean_yolo11n/weights/best.pt"


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
        (
            "share/" + package_name + "/models/netclean_yolo11n/weights",
            [model_file],
        ),
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
            "vision1_node_v2 = nc_vision.vision1_node_v2:main",
            "vision2_node_v2 = nc_vision.vision2_node_v2:main",
        ],
    },
)
