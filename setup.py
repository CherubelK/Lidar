"""
Setup script for Unitree L2 LiDAR Trail Mapping System
"""

from setuptools import setup, find_packages

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setup(
    name="unitree-lidar-trail-mapper",
    version="0.1.0",
    author="Your Name",
    author_email="your.email@example.com",
    description="3D trail mapping system using Unitree L2 LiDAR",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/yourusername/unitree-lidar-trail-mapper",
    packages=find_packages(),
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Developers",
        "Topic :: Scientific/Engineering :: GIS",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
    ],
    python_requires=">=3.8",
    install_requires=[
        "numpy>=1.24.0",
        "pandas>=2.0.0",
        "open3d>=0.17.0",
        "scipy>=1.10.0",
        "matplotlib>=3.7.0",
        "plotly>=5.14.0",
        "h5py>=3.8.0",
        "pillow>=9.5.0",
        "pyyaml>=6.0",
        "python-dotenv>=1.0.0",
        "tqdm>=4.65.0",
    ],
    extras_require={
        "dev": [
            "pytest>=7.3.0",
            "pytest-cov>=4.1.0",
            "black>=23.0.0",
            "flake8>=6.0.0",
            "mypy>=1.0.0",
        ],
    },
)