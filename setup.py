import os
import re
import sys
import platform
import subprocess
import socket
import time
import shutil
import glob

from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext
from distutils.version import LooseVersion

import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--optix-home', type=str, default=None)
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

        if platform.system() == "Windows":
            cmake_version = LooseVersion(re.search(r'version\s*([\d.]+)', out.decode()).group(1))
            if cmake_version < '3.12.0':
                raise RuntimeError("CMake >= 3.12.0 is required on Windows")

        for ext in self.extensions:
            self.build_extension(ext)

    def build_extension(self, ext):
        original_full_path = self.get_ext_fullpath(ext.name)
        extdir = os.path.abspath(os.path.dirname(original_full_path))
        extdir = os.path.join(extdir, self.distribution.get_name(), "core")
        cmake_args = ['-DCMAKE_LIBRARY_OUTPUT_DIRECTORY=' + extdir, '-DPYTHON_EXECUTABLE=' + sys.executable]

        if args.optix_home is not None:
            cmake_args.append('-DOPTIX_HOME=' + args.optix_home)

        cfg = 'Release'
        build_args = ['--config', cfg]

        cmake_args += ['-DCMAKE_BUILD_TYPE=' + cfg]
        build_args += ['--', '-j8']

        env = os.environ.copy()
        env['CXXFLAGS'] = '{} -DVERSION_INFO=\\"{}\\"'.format(env.get('CXXFLAGS', ''),
                                                              self.distribution.get_version())

        if not os.path.exists(self.build_temp):
            os.makedirs(self.build_temp)
        subprocess.check_call(['cmake', ext.sourcedir] + cmake_args, cwd=self.build_temp, env=env)
        subprocess.check_call(['cmake', '--build', '.'] + build_args, cwd=self.build_temp)

        glsl_target_path = os.path.join(self.build_lib, 'sapien', 'glsl_shader')
        if os.path.exists(glsl_target_path):
            shutil.rmtree(glsl_target_path)
        shutil.copytree(os.path.join(self.build_temp, 'glsl_shader'), glsl_target_path)

        if args.optix_home:
            ptx_target_path = os.path.join(self.build_lib, 'sapien', 'ptx')
            print(ptx_target_path)
            if os.path.exists(ptx_target_path):
                shutil.rmtree(ptx_target_path)
            shutil.copytree(os.path.join(self.build_temp, 'ptx'), ptx_target_path)


def check_version_info():
    try:
        git_revision = subprocess.check_output(["git", "rev-parse", "HEAD"]).decode("utf-8").split("\n")[0]
        git_branch = subprocess.check_output(["git", "rev-parse", "--abbrev-ref",
                                              "HEAD"]).decode("utf-8").split("\n")[0]
    except (subprocess.CalledProcessError, OSError):
        git_revision = ""
        git_branch = "non-git"

    def read_version():
        with open("python/VERSION") as f:
            return f.readline().strip()

    build_datetime = time.strftime("%a, %d %b %Y %H:%M:%S +0000", time.gmtime())
    version_number = read_version()

    hostname = socket.gethostname()

    sys.stdout.write(f"====================Version Data====================\n"
                     f"Git Revision Number: {git_revision}\n"
                     f"Git Branch: {git_branch}\n"
                     f"Build Datatime: {build_datetime}\n"
                     f"Version Number: {version_number}\n"
                     f"Host Name: {hostname}\n"
                     f"====================================================\n\n")

    return git_revision, git_branch, build_datetime, version_number, hostname


# Read requirements.txt
def read_requirements():
    with open('python/requirements.txt', 'r') as f:
        lines = f.readlines()
    install_requires = [line.strip() for line in lines if line]
    return install_requires


# Data files for packaging
project_python_home_dir = os.path.join("python", "py_package")

assets_target = os.path.join(project_python_home_dir, "assets")
if not os.path.exists(assets_target):
    shutil.copytree(os.path.join("./assets"), assets_target)

sapien_data = ["glsl_shader/*/*"]
sapien_data.extend(glob.glob("assets/robot/**", recursive=True))
package_data = {
    "sapien": sapien_data,
}

name = 'sapien'

setup(name=name,
      version=check_version_info()[3],
      author='SAPIEN Team',
      author_email='sapienaicontact@gmail.com',
      description=['SAPIEN: A SimulAted Parted based Interactive ENvironment'],
      classifiers=[
          "Operating System :: POSIX :: Linux",
          "Programming Language :: C++",
          "Programming Language :: Python :: 3.6",
          "Programming Language :: Python :: 3.7",
          "Topic :: Education",
          "Topic :: Software Development :: Libraries :: Python Modules",
          "Topic :: Utilities",
      ],
      license="MIT",
      ext_modules=[CMakeExtension('sapien')],
      install_requires=read_requirements(),
      long_description=open("readme.md").read(),
      cmdclass=dict(build_ext=CMakeBuild),
      zip_safe=False,
      packages=["sapien", "sapien.env", "sapien.core", "sapien.asset", "sapien.extension"],
      keywords="robotics simulator dataset articulation partnet",
      url="sapien.ucsd.edu",
      project_urls={"Documentation": "sapien.ucsd.edu/docs"},
      package_data=package_data,
      package_dir={"sapien": project_python_home_dir})
