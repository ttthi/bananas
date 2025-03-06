from setuptools import setup, find_packages

setup(
    name='ply_processor',            # The package name
    version='0.1.0',
    packages=find_packages(),        # This find the ply_processor package
    install_requires=[
        'open3d==0.19.0',                    # Add dependencies
        'numpy==1.26.4',
        'matplotlib==3.9.2',
        'matplotlib-inline==0.1.7'
    ],
    author='Chen Xu',
    author_email='cxu.self@gmail.com',
    description='A module for processing point clouds with plane fitting and bounding boxes',
    classifiers=[
        'Programming Language :: Python :: 3',
        'Operating System :: OS Independent',
    ],
    python_requires='>=3.6',
)
