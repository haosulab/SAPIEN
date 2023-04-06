from setuptools import setup, find_packages, Extension
from setuptools.command.build_ext import build_ext
import os
import glob
import sapien


def get_extensions():
    this_dir = os.path.dirname(os.path.abspath(__file__))
    extensions_dir = os.path.join(this_dir, "example_extension")

    sources = glob.glob(os.path.join(extensions_dir, "*.cpp"))

    define_macros = []
    extra_compile_args = ['-std=c++17']
    libraries = []

    sources = [os.path.join(extensions_dir, s) for s in sources]

    include_dirs = [
        os.path.join(sapien.__path__[0], "include"),
        os.path.join(sapien.__path__[0], "include", "physx"),
        os.path.join(sapien.__path__[0], "include", "pxshared"),
    ]  # SAPIEN include path
    include_dirs += [extensions_dir, os.path.join(extensions_dir, "include")]
    print(include_dirs)

    ext_modules = [
        Extension(
            name="sapien_example_extension",
            sources=sources,
            include_dirs=include_dirs,
            libraries=libraries,
            define_macros=define_macros,
            extra_compile_args=extra_compile_args,
            language="c++",
        )
    ]
    return ext_modules


setup(
    name="sapien_example_extension",
    version="0.0.1",
    author="",
    author_email="",
    description="",
    install_requires=["sapien"],
    python_requires=">=3.7",
    url="",
    packages=find_packages(include=["example_extension"], exclude=["tests"]),
    long_description="",
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: Apache Software License",
        "Operating System :: OS Independent",
        "Intended Audience :: Science/Research",
        "Topic :: Scientific/Engineering :: Artificial Intelligence",
    ],
    ext_modules=get_extensions(),
)
