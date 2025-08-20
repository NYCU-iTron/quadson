from setuptools import setup, find_packages

setup(
    name="quadson_py",
    version="0.1.0",
    package_dir={"": "src"},
    packages=find_packages(where="src"),
    python_requires="==3.10.*",
    install_requires=[
        "stable-baselines3==2.7.0",
        "numpy==2.2.6",
        "cloudpickle==3.1.1",
        "gymnasium==1.2.0",
        # torch+cu128 has to be installed via URL
        # see https://pytorch.org/get-started/locally/
        # Example for pip with CUDA 12.8
        # "torch==2.8.0+cu128 --index-url https://download.pytorch.org/whl/cu128",
    ],
)