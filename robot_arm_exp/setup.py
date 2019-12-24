from setuptools import setup, find_packages
import glob

# Force platform specific wheel
try:
    from wheel.bdist_wheel import bdist_wheel as _bdist_wheel


    # https://stackoverflow.com/a/45150383/1255535
    class bdist_wheel(_bdist_wheel):
        def finalize_options(self):
            _bdist_wheel.finalize_options(self)
            self.root_is_pure = False
except ImportError:
    print('Warning: cannot import "wheel" package to build platform-specific wheel')
    print('Install the "wheel" package to fix this warning')
    bdist_wheel = None

cmd_class = {'bdist_wheel': bdist_wheel} if bdist_wheel is not None else dict()

# Read requirements.txt
with open('requirements.txt', 'r') as f:
    lines = f.readlines()
install_requires = [line.strip() for line in lines if line]

# Data files for packaging
data_files = [
]

setup(
    author='SAPIEN Team',
    author_email='sapien@ucsd.edu',
    classifiers=[
        # https://pypi.org/pypi?%3Aaction=list_classifiers
        "Operating System :: POSIX :: Linux",
        "Programming Language :: C++",
        "Programming Language :: Python :: 3.6",
        "Programming Language :: Python :: 3.7",
        "Topic :: Education",
        "Topic :: Software Development :: Libraries :: Python Modules",
        "Topic :: Utilities",
    ],
    description=[
        "SAPIEN is a robotics simulator"
    ],
    cmdclass=cmd_class,
    install_requires=install_requires,
    include_package_data=True,
    data_files=data_files,
    keywords="robotics simulator dataset articulation part-net",
    license="MIT",
    long_description="TODO",
    # Name of the package on PyPI
    name="sapyen",
    packages=["sapien"],
    version='0.0.1',
    zip_safe=False,
)
