from setuptools import setup, find_packages

setup(
    name='LGR_MPC',  # Your package name
    version='0.1.0',  # Initial version
    description='A Python package for LGR-based Model Predictive Control.',  # Short description
    author='Saeid Bayat',  # Your name
    author_email='saeidb@umich.edu',  # Your email
    url='https://github.com/saeidb71/LGR_MPC.git',  # Replace with your GitHub repository URL
    packages=find_packages(),  # Automatically finds all sub-packages
    include_package_data=True,  # Includes files listed in MANIFEST.in
    install_requires=[
        'numpy',
        'scipy',
        'matplotlib',
    ],
    python_requires='>=3.10',  # Specify Python version compatibility
    classifiers=[
        'Programming Language :: Python :: 3',
        'License :: OSI Approved :: MIT License',  # Update if you use a different license
        'Operating System :: OS Independent',
    ],
)
