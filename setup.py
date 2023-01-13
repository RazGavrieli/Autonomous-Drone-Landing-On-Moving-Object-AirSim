import pathlib
from setuptools import setup, find_packages

def parse_requirements_file(filename="requirements.txt"):
    with open(filename) as fid:
        requires = [lin.strip() for lin in fid.readlines()]
    return requires

HERE = pathlib.Path(__file__).parent
README = (HERE/"README.md").read_text()
REQUIRES = parse_requirements_file()

setup(
    name="airsim_autonomous_landing", 
    version="0.1.0", 
    description="easy to use autonomous landing package for airsim",
    long_description=README,
    long_description_content_type="text/markdown",
    url="https://github.com/RazGavrieli/Autonomous-Drone-Landing-On-Moving-Object-AirSim",
    author="Raz Gavrieli", 
    author_email="razgavrieli@gmail.com",
    ackages=find_packages(exclude=("tests",)),
    include_package_data=True,
    install_requires = REQUIRES,
    license="MIT"
    )


