import os
import re
import sys
import platform
import subprocess
import socket
import time
import shutil
import argparse

from setuptools import setup, Extension
from setuptools.command.build import build
from setuptools.command.build_ext import build_ext
from wheel.bdist_wheel import bdist_wheel
from distutils.command.bdist import bdist


parser = argparse.ArgumentParser()
parser.add_argument("--debug", action="store_true")
parser.add_argument("--profile", action="store_true")
parser.add_argument(
    "--sapien-only", action="store_true", help="build sapien without python binding"
)
parser.add_argument(
    "--pybind-only",
    action="store_true",
    help="build python binding assuming sapien is already built",
)
parser.add_argument(
    "--build-dir",
    type=str,
    default="sapien_build",
    help="directory to put build files",
)

args, unknown = parser.parse_known_args()
sys.argv = [sys.argv[0]] + unknown


def build_sapien(sapien_source_dir, sapien_build_dir):
    build_dir = os.path.join(sapien_build_dir, "_sapien_build")
    install_dir = os.path.join(sapien_build_dir, "_sapien_install")
    os.makedirs(build_dir, exist_ok=True)
    os.makedirs(install_dir, exist_ok=True)

    cmake_args = []

    if args.debug:
        cfg = "Debug"
    else:
        cfg = "Release"
    build_args = ["--config", cfg]
    if args.profile:
        cmake_args += ["-DSAPIEN_PROFILE=ON"]
    else:
        cmake_args += ["-DSAPIEN_PROFILE=OFF"]

    if os.environ.get("CUDA_PATH") is not None:
        cmake_args += ["-DSAPIEN_CUDA=ON"]
    else:
        cmake_args += ["-DSAPIEN_CUDA=OFF"]

    cmake_args += ["-DCMAKE_BUILD_TYPE=" + cfg]
    cmake_args += ["-DCMAKE_INSTALL_PREFIX=" + install_dir]

    env = os.environ.copy()
    subprocess.check_call(
        ["cmake", sapien_source_dir] + cmake_args, cwd=build_dir, env=env
    )
    subprocess.check_call(
        ["cmake", "--build", ".", "--target", "install"] + build_args,
        cwd=build_dir,
    )


class sapien_bdist(bdist):
    def initialize_options(self):
        super().initialize_options()
        self.bdist_base = os.path.join(os.path.dirname(__file__), args.build_dir)


class sapien_build(build):
    def initialize_options(self):
        super().initialize_options()
        self.build_base = os.path.join(os.path.dirname(__file__), args.build_dir)


class CMakeExtension(Extension):
    def __init__(self, name, sourcedir="./"):
        Extension.__init__(self, name, sources=[])
        self.sourcedir = os.path.abspath(sourcedir)


class CMakeBuild(build_ext):
    def initialize_options(self):
        super().initialize_options()
        self.sapien_build_dir = os.path.join(
            os.path.abspath(os.path.dirname(__file__)), args.build_dir
        )
        self.build_base = self.sapien_build_dir

    def run(self):
        try:
            out = subprocess.check_output(["cmake", "--version"])
        except OSError:
            raise RuntimeError(
                "CMake must be installed to build the following extensions: "
                + ", ".join(e.name for e in self.extensions)
            )

        for ext in self.extensions:
            self.build_extension(ext)

    def build_pinocchio(self, ext):
        sapien_install_dir = os.path.join(self.sapien_build_dir, "_sapien_install")
        build_dir = os.path.join(self.sapien_build_dir, "_pinocchio_build")
        os.makedirs(build_dir, exist_ok=True)
        original_full_path = self.get_ext_fullpath(ext.name)
        extdir = os.path.abspath(os.path.dirname(original_full_path))
        extdir = os.path.join(extdir, self.distribution.get_name(), "core")
        cmake_args = [
            "-DCMAKE_LIBRARY_OUTPUT_DIRECTORY=" + extdir,
            "-DPYTHON_EXECUTABLE=" + sys.executable,
        ]
        if args.debug:
            cfg = "Debug"
        else:
            cfg = "Release"
        build_args = ["--config", cfg]
        cmake_args += ["-DCMAKE_BUILD_TYPE=" + cfg]
        cmake_args += [
            "-Dsapien_DIR=" + os.path.join(sapien_install_dir, "lib/cmake/sapien")
        ]
        env = os.environ.copy()
        subprocess.check_call(
            ["cmake", os.path.join(ext.sourcedir, "pinocchio")] + cmake_args,
            cwd=build_dir,
            env=env,
        )
        subprocess.check_call(
            ["cmake", "--build", ".", "--target", "pysapien_pinocchio"] + build_args,
            cwd=build_dir,
        )

    def build_pybind(self, ext):
        sapien_install_dir = os.path.join(self.sapien_build_dir, "_sapien_install")

        os.makedirs(self.build_temp, exist_ok=True)
        original_full_path = self.get_ext_fullpath(ext.name)
        extdir = os.path.abspath(os.path.dirname(original_full_path))
        extdir = os.path.join(extdir, self.distribution.get_name(), "core")
        cmake_args = [
            "-DCMAKE_LIBRARY_OUTPUT_DIRECTORY=" + extdir,
            "-DPYTHON_EXECUTABLE=" + sys.executable,
        ]
        if args.debug:
            cfg = "Debug"
        else:
            cfg = "Release"
        build_args = ["--config", cfg]
        cmake_args += ["-DCMAKE_BUILD_TYPE=" + cfg]
        cmake_args += [
            "-Dsapien_DIR=" + os.path.join(sapien_install_dir, "lib/cmake/sapien")
        ]
        env = os.environ.copy()
        subprocess.check_call(
            ["cmake", os.path.join(ext.sourcedir, "python")] + cmake_args,
            cwd=self.build_temp,
            env=env,
        )
        subprocess.check_call(
            ["cmake", "--build", ".", "--target", "pysapien"] + build_args,
            cwd=self.build_temp,
        )

        include_path = os.path.join(self.build_lib, "sapien", "include")
        source_include_path = os.path.join(sapien_install_dir, "include")
        if os.path.exists(include_path):
            shutil.rmtree(include_path)
        shutil.copytree(source_include_path, include_path)

    def copy_assets(self, ext):
        vulkan_shader_path = os.path.join(self.build_lib, "sapien", "vulkan_shader")
        source_path = os.path.join(ext.sourcedir, "vulkan_shader")
        if os.path.exists(vulkan_shader_path):
            shutil.rmtree(vulkan_shader_path)
        assert os.path.exists(source_path)
        shutil.copytree(source_path, vulkan_shader_path)

        # provide Vulkan libraries for linux
        if platform.system() == "Linux":
            vulkan_library_path = os.path.join(
                self.build_lib, "sapien", "vulkan_library"
            )
            source_path = os.path.join(ext.sourcedir, "vulkan_library")
            if os.path.exists(vulkan_library_path):
                shutil.rmtree(vulkan_library_path)
            assert os.path.exists(source_path)
            shutil.copytree(source_path, vulkan_library_path)

        sensor_assets_path = os.path.join(self.build_lib, "sapien", "sensor", "assets")
        source_patterns_path = os.path.join(
            ext.sourcedir, "python", "py_package", "sensor", "assets", "patterns"
        )
        if os.path.exists(sensor_assets_path):
            shutil.rmtree(sensor_assets_path)
        os.makedirs(sensor_assets_path)
        shutil.copytree(
            source_patterns_path, os.path.join(sensor_assets_path, "patterns")
        )

    def build_extension(self, ext):
        if platform.system() == "Linux":
            self.build_pinocchio(ext)
        self.build_pybind(ext)
        self.copy_assets(ext)


def check_version_info():
    try:
        git_revision = (
            subprocess.check_output(["git", "rev-parse", "HEAD"])
            .decode("utf-8")
            .split("\n")[0]
        )
        git_branch = (
            subprocess.check_output(["git", "rev-parse", "--abbrev-ref", "HEAD"])
            .decode("utf-8")
            .split("\n")[0]
        )
    except (subprocess.CalledProcessError, OSError):
        git_revision = ""
        git_branch = "non-git"

    build_datetime = time.strftime("%a, %d %b %Y %H:%M:%S +0000", time.gmtime())
    with open("python/VERSION") as f:
        version_number = f.readline().strip()

    hostname = socket.gethostname()

    sys.stdout.write(
        "====================Version Data====================\n"
        "Git Revision Number: {}\n"
        "Git Branch: {}\n"
        "Build Datetime: {}\n"
        "Version Number: {}\n"
        "Host Name: {}\n"
        "====================================================\n\n".format(
            git_revision, git_branch, build_datetime, version_number, hostname
        )
    )

    return git_revision, git_branch, build_datetime, version_number, hostname


# Read requirements.txt
def read_requirements():
    with open("python/requirements.txt", "r") as f:
        lines = f.readlines()
    install_requires = [line.strip() for line in lines if line]
    return install_requires


# Data files for packaging
project_python_home_dir = os.path.join("python", "py_package")
package_data = {
    "sapien.core": [
        "__init__.pyi",
        "pysapien/__init__.pyi",
        "pysapien/renderer/__init__.pyi",
        "pysapien/dlpack/__init__.pyi",
    ]
}


if not args.pybind_only:
    # build SAPIEN
    source_dir = os.path.dirname(__file__)
    build_sapien(source_dir, os.path.join(source_dir, args.build_dir))

if args.sapien_only:
    exit(0)


setup(
    name="sapien",
    version=check_version_info()[3],
    author="Sapien",
    python_requires=">=3.7",
    author_email="sapienaicontact@gmail.com",
    description=["SAPIEN: A SimulAted Parted based Interactive ENvironment"],
    classifiers=[
        "Operating System :: POSIX :: Linux",
        "Intended Audience :: Developers",
        "Intended Audience :: Education",
        "Intended Audience :: Other Audience",
        "Intended Audience :: Science/Research",
        "License :: OSI Approved :: MIT License",
        "Natural Language :: English",
        "Framework :: Robot Framework :: Tool",
        "Topic :: Games/Entertainment :: Simulation",
        "Programming Language :: C++",
        "Programming Language :: Python :: 3.7",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Topic :: Education",
        "Topic :: Software Development :: Libraries :: Python Modules",
        "Topic :: Utilities",
    ],
    license="MIT",
    ext_modules=[CMakeExtension("sapien")],
    install_requires=read_requirements(),
    long_description=open("readme.md").read(),
    long_description_content_type="text/markdown",
    cmdclass=dict(build=sapien_build, build_ext=CMakeBuild, bdist=sapien_bdist),
    zip_safe=False,
    packages=[
        "sapien",
        "sapien.core",
        "sapien.asset",
        "sapien.example",
        "sapien.utils",
        "sapien.utils.viewer",
        "sapien.sensor",
    ],
    keywords="robotics simulator dataset articulation partnet",
    url="https://sapien.ucsd.edu",
    project_urls={"Documentation": "https://sapien.ucsd.edu/docs"},
    package_data=package_data,
    package_dir={"sapien": project_python_home_dir},
    scripts=[],
)
