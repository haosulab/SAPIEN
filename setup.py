import os
import re
import sys
import platform
import subprocess
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
parser.add_argument(
    "--get-version",
    action="store_true",
    help="print version string and exit immediately",
)

args, unknown = parser.parse_known_args()
sys.argv = [sys.argv[0]] + unknown


def generate_version():
    try:
        git_revision = (
            subprocess.check_output(["git", "rev-parse", "HEAD"])
            .decode("utf-8")
            .split("\n")[0]
        )
    except (subprocess.CalledProcessError, OSError):
        git_revision = ""

    try:
        git_revision_short = (
            subprocess.check_output(["git", "rev-parse", "--short", "HEAD"])
            .decode("utf-8")
            .split("\n")[0]
        )
    except (subprocess.CalledProcessError, OSError):
        git_revision_short = ""

    try:
        git_branch = (
            subprocess.check_output(["git", "rev-parse", "--abbrev-ref", "HEAD"])
            .decode("utf-8")
            .split("\n")[0]
        )
    except (subprocess.CalledProcessError, OSError):
        git_branch = "non-git"

    try:
        git_tag = (
            subprocess.check_output(
                ["git", "describe", "--tags", "--exact-match", "HEAD"],
                stderr=subprocess.DEVNULL,
            )
            .decode("utf-8")
            .split("\n")[0]
        )
    except (subprocess.CalledProcessError, OSError):
        git_tag = ""

    build_datetime = time.strftime("%a, %d %b %Y %H:%M:%S +0000", time.gmtime())
    build_datetime_short = time.strftime("%Y%m%d", time.gmtime())

    if git_branch == "master":
        assert git_tag is not None

    if git_tag in ["", "nightly"]:
        # no meaning tag
        with open(
            os.path.join(os.path.dirname(__file__), "python", "VERSION"), "r"
        ) as f:
            base_version = f.readline().strip()
        version = (
            base_version + ".dev" + build_datetime_short + "+" + git_revision_short
        )
    else:
        version = git_tag

    return version


version = generate_version()

if args.get_version:
    print(version)
    exit(0)


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

    cmake_args += [
        f"-DCMAKE_BUILD_TYPE={cfg}",
        f"-DCMAKE_INSTALL_PREFIX={install_dir}",
        "-DCMAKE_MSVC_RUNTIME_LIBRARY=MultiThreaded",
    ]

    if sys.platform == 'win32':
        cmake_args += [f"-DBUILD_TESTING=Off"]

    deps_dir = os.path.join(sapien_build_dir, "_sapien_deps")
    cmake_args += [f"-DFETCHCONTENT_BASE_DIR={deps_dir}"]

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
        extdir = os.path.join(extdir, self.distribution.get_name())
        cmake_args = [
            f"-DCMAKE_LIBRARY_OUTPUT_DIRECTORY=$<1:{extdir}>",
            f"-DPYTHON_EXECUTABLE={sys.executable}",
            f"-DCMAKE_MSVC_RUNTIME_LIBRARY=MultiThreaded",
        ]

        deps_dir = os.path.join(self.sapien_build_dir, "_sapien_deps")
        cmake_args += [f"-DFETCHCONTENT_BASE_DIR={deps_dir}"]

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
        extdir = os.path.join(extdir, self.distribution.get_name())
        cmake_args = [
            f"-DCMAKE_LIBRARY_OUTPUT_DIRECTORY=$<1:{extdir}>",
            f"-DPYTHON_EXECUTABLE={sys.executable}",
            f"-DCMAKE_MSVC_RUNTIME_LIBRARY=MultiThreaded",
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

        if platform.system() == "Windows":
            bindir = os.path.join(sapien_install_dir, "bin")
            for f in os.listdir(bindir):
                if f.endswith("dll"):
                    shutil.copy(os.path.join(bindir, f), extdir)

        oidn_library_path = os.path.join(extdir, "oidn_library")
        if os.path.exists(oidn_library_path):
            shutil.rmtree(oidn_library_path)
        os.makedirs(oidn_library_path, exist_ok=True)

        # provide oidn for linux
        if platform.system() == "Linux":
            for folder in ["lib", "lib64"]:
                library_dir = os.path.join(sapien_install_dir, folder)
                if not os.path.exists(library_dir):
                    continue
                print("copy library from", library_dir)
                for lib in os.listdir(library_dir):
                    if lib in [
                        "libOpenImageDenoise.so.2.0.1",
                        "libOpenImageDenoise_core.so.2.0.1",
                        "libOpenImageDenoise_device_cuda.so.2.0.1",
                    ]:
                        shutil.copy(os.path.join(library_dir, lib), oidn_library_path)

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
        with open(os.path.join(self.build_lib, "sapien", "version.py"), "w") as f:
            f.write('__version__="{}"'.format(version))

    def build_extension(self, ext):
        if platform.system() == "Linux":
            self.build_pinocchio(ext)
        self.build_pybind(ext)
        self.copy_assets(ext)


# Read requirements.txt
def read_requirements():
    with open("python/requirements.txt", "r") as f:
        lines = f.readlines()
    install_requires = [line.strip() for line in lines if line]
    return install_requires


# Data files for packaging
project_python_home_dir = os.path.join("python", "py_package")
package_data = {
    "sapien": [
        "__init__.pyi",
        "pysapien/__init__.pyi",
        "pysapien/simsense.pyi",
        "pysapien/physx.pyi",
        "pysapien/render.pyi",
        "pysapien/math.pyi",
        "pysapien/internal_renderer.pyi",
    ],
}


if not args.pybind_only:
    # build SAPIEN
    source_dir = os.path.abspath(os.path.dirname(__file__))
    build_sapien(source_dir, os.path.join(source_dir, args.build_dir))

if args.sapien_only:
    exit(0)

setup(
    name="sapien",
    version=version,
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
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Programming Language :: Python :: 3.12",
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
        "sapien.package",
        "sapien.physx",
        "sapien.internal_renderer",
        "sapien.render",
        "sapien.wrapper",
        "sapien.core",
        "sapien.wrapper.urchin",
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
    scripts=["python/py_package/scripts/sapien"],
)
