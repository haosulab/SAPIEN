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
from setuptools.command.build_ext import build_ext

parser = argparse.ArgumentParser()
parser.add_argument('--debug', action='store_true')
parser.add_argument('--profile', action='store_true')
args, unknown = parser.parse_known_args()
sys.argv = [sys.argv[0]] + unknown


class CMakeExtension(Extension):
    def __init__(self, name, sourcedir='./'):
        Extension.__init__(self, name, sources=[])
        self.sourcedir = os.path.abspath(sourcedir)


class CMakeBuild(build_ext):
    def run(self):
        try:
            out = subprocess.check_output(['cmake', '--version'])
        except OSError:
            raise RuntimeError("CMake must be installed to build the following extensions: " +
                               ", ".join(e.name for e in self.extensions))

        for ext in self.extensions:
            self.build_extension(ext)

    def build_extension(self, ext):
        original_full_path = self.get_ext_fullpath(ext.name)
        extdir = os.path.abspath(os.path.dirname(original_full_path))
        extdir = os.path.join(extdir, self.distribution.get_name(), "core")
        cmake_args = ['-DCMAKE_LIBRARY_OUTPUT_DIRECTORY=' + extdir, '-DPYTHON_EXECUTABLE=' + sys.executable]

        if args.debug:
            cfg = 'Debug'
        else:
            cfg = 'Release'
        build_args = ['--config', cfg]
        if args.profile:
            cmake_args += ['-DSVULKAN2_PROFILE=ON']
            cmake_args += ['-DSAPIEN_PROFILE=ON']

        if platform.system() != 'Darwin':
            cmake_args += ['-DSAPIEN_DLPACK_INTEROP=ON', '-DSVULKAN2_CUDA_INTEROP=ON']

        cmake_args += ['-DCMAKE_BUILD_TYPE=' + cfg]
        build_args += ['--', '-j8']

        env = os.environ.copy()
        env['CXXFLAGS'] = '{} -DVERSION_INFO=\\"{}\\"'.format(env.get('CXXFLAGS', ''),
                                                              self.distribution.get_version())

        if not os.path.exists(self.build_temp):
            os.makedirs(self.build_temp)
        subprocess.check_call(['cmake', ext.sourcedir] + cmake_args, cwd=self.build_temp, env=env)
        subprocess.check_call(['cmake', '--build', '.', "--target", "pysapien"] + build_args, cwd=self.build_temp)

        vulkan_shader_path = os.path.join(self.build_lib, 'sapien', 'vulkan_shader')
        source_path = os.path.join(ext.sourcedir, 'vulkan_shader')
        if os.path.exists(vulkan_shader_path):
            shutil.rmtree(vulkan_shader_path)
        assert os.path.exists(source_path)
        shutil.copytree(source_path, vulkan_shader_path)

        vulkan_icd_path = os.path.join(self.build_lib, 'sapien', 'vulkan_icd')
        source_path = os.path.join(ext.sourcedir, 'vulkan_icd')
        if os.path.exists(vulkan_icd_path):
            shutil.rmtree(vulkan_icd_path)
        assert os.path.exists(source_path)
        shutil.copytree(source_path, vulkan_icd_path)

        kuafu_shader_path = os.path.join(self.build_lib, 'sapien', 'kuafu_assets', 'shaders')
        source_path = os.path.join(ext.sourcedir, '3rd_party/kuafu/resources/shaders')
        if os.path.exists(kuafu_shader_path):
            shutil.rmtree(kuafu_shader_path)
        assert os.path.exists(source_path)
        shutil.copytree(source_path, kuafu_shader_path)

        sensor_assets_path = os.path.join(self.build_lib, 'sapien', 'sensor', 'assets')
        source_patterns_path = os.path.join(ext.sourcedir, '3rd_party/kuafu/resources/patterns')
        if os.path.exists(sensor_assets_path):
            shutil.rmtree(sensor_assets_path)
        os.makedirs(sensor_assets_path)
        shutil.copytree(source_patterns_path, os.path.join(sensor_assets_path, 'patterns'))

        include_path = os.path.join(self.build_lib, 'sapien', 'include')
        source_include_path = os.path.join(ext.sourcedir, 'include')
        if os.path.exists(include_path):
            shutil.rmtree(include_path)
        shutil.copytree(source_include_path, include_path)

        include_path = os.path.join(self.build_lib, 'sapien', 'include', 'pybind11')
        source_include_path = os.path.join(ext.sourcedir, '3rd_party', 'pybind11', 'include', 'pybind11')
        if os.path.exists(include_path):
            shutil.rmtree(include_path)
        shutil.copytree(source_include_path, include_path)

        include_path = os.path.join(self.build_lib, 'sapien', 'include', 'dlpack')
        source_include_path = os.path.join(ext.sourcedir, '3rd_party', 'dlpack', 'include', 'dlpack')
        if os.path.exists(include_path):
            shutil.rmtree(include_path)
        shutil.copytree(source_include_path, include_path)

        include_path = os.path.join(self.build_lib, 'sapien', 'include', 'svulkan2')
        source_include_path = os.path.join(ext.sourcedir, '3rd_party', 'sapien-vulkan-2', 'include', 'svulkan2')
        if os.path.exists(include_path):
            shutil.rmtree(include_path)
        shutil.copytree(source_include_path, include_path)

        include_path = os.path.join(self.build_lib, 'sapien', 'include', 'kuafu')
        source_include_path = os.path.join(ext.sourcedir, '3rd_party', 'kuafu', 'include')
        if os.path.exists(include_path):
            shutil.rmtree(include_path)
        shutil.copytree(source_include_path, include_path)

        shutil.copy(os.path.join(ext.sourcedir, '3rd_party', 'tinyxml2', 'tinyxml2.h'),
                    os.path.join(self.build_lib, 'sapien', 'include', 'tinyxml2.h'))

        include_path = os.path.join(self.build_lib, 'sapien', 'include', 'physx')
        source_include_path = os.path.join(ext.sourcedir, '..', 'PhysX', 'physx', 'include')
        if os.path.exists(include_path):
            shutil.rmtree(include_path)
        shutil.copytree(source_include_path, include_path)

        include_path = os.path.join(self.build_lib, 'sapien', 'include', 'pxshared')
        source_include_path = os.path.join(ext.sourcedir, '..', 'PhysX', 'pxshared', 'include')
        if os.path.exists(include_path):
            shutil.rmtree(include_path)
        shutil.copytree(source_include_path, include_path)

        def copy_system_header(name):
            system_include_path = ['/usr/local/include', '/usr/include']
            for p in system_include_path:
                source_include_path = os.path.join(p, name)
                if os.path.isdir(p):
                    break
            else:
                raise ValueError(f'cannot find library {name}')
            include_path = os.path.join(self.build_lib, 'sapien', 'include', 'spdlog')
            if os.path.exists(include_path):
                shutil.rmtree(include_path)
            shutil.copytree(source_include_path, include_path)

        copy_system_header('spdlog')


def check_version_info():
    try:
        git_revision = subprocess.check_output(["git", "rev-parse", "HEAD"]).decode("utf-8").split("\n")[0]
        git_branch = subprocess.check_output(["git", "rev-parse", "--abbrev-ref",
                                              "HEAD"]).decode("utf-8").split("\n")[0]
    except (subprocess.CalledProcessError, OSError):
        git_revision = ""
        git_branch = "non-git"

    build_datetime = time.strftime("%a, %d %b %Y %H:%M:%S +0000", time.gmtime())
    with open("python/VERSION") as f:
        version_number = f.readline().strip()

    hostname = socket.gethostname()

    sys.stdout.write("====================Version Data====================\n"
                     "Git Revision Number: {}\n"
                     "Git Branch: {}\n"
                     "Build Datetime: {}\n"
                     "Version Number: {}\n"
                     "Host Name: {}\n"
                     "====================================================\n\n".format(git_revision, git_branch,
                                                                                       build_datetime, version_number,
                                                                                       hostname))

    return git_revision, git_branch, build_datetime, version_number, hostname


# Read requirements.txt
def read_requirements():
    with open('python/requirements.txt', 'r') as f:
        lines = f.readlines()
    install_requires = [line.strip() for line in lines if line]
    return install_requires


# Data files for packaging
project_python_home_dir = os.path.join("python", "py_package")
package_data = {
    "sapien.core": ["__init__.pyi", "pysapien/__init__.pyi", "pysapien/renderer/__init__.pyi"]
}

setup(name="sapien",
      version=check_version_info()[3],
      author='Sapien',
      python_requires='>=3.7',
      author_email='sapienaicontact@gmail.com',
      description=['SAPIEN: A SimulAted Parted based Interactive ENvironment'],
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
      ext_modules=[CMakeExtension('sapien')],
      install_requires=read_requirements(),
      long_description=open("readme.md").read(),
      long_description_content_type="text/markdown",
      cmdclass=dict(build_ext=CMakeBuild),
      zip_safe=False,
      packages=["sapien", "sapien.core", "sapien.asset", "sapien.example", "sapien.utils", "sapien.sensor"],
      keywords="robotics simulator dataset articulation partnet",
      url="https://sapien.ucsd.edu",
      project_urls={"Documentation": "https://sapien.ucsd.edu/docs"},
      package_data=package_data,
      package_dir={"sapien": project_python_home_dir})
