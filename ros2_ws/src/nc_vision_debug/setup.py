from glob import glob
from setuptools import find_packages, setup


package_name = "nc_vision_debug"


setup(
    name=package_name,
    version="0.7.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/examples", glob("examples/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="NetClean Team",
    maintainer_email="rokey@example.com",
    description="NetClean Vision debug popup nodes",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "vision1_debug_popup = nc_vision_debug.vision1_debug_popup_node:main",
            "vision2_debug_popup = nc_vision_debug.vision2_debug_popup_node:main",
            "vision1_debug_popup_v3 = nc_vision_debug.vision1_debug_popup_v3_node:main",
            "vision2_debug_popup_v3 = nc_vision_debug.vision2_debug_popup_v3_node:main",
        ],
    },
)
