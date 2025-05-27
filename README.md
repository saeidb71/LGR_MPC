# A Practical Open-Source Approach to Model Predictive Control Using the Legendre-Gauss-Radau Pseudospectral Method

## Overview

This open-source Python package provides a user-friendly framework for solving Model Predictive Control (MPC) problems using the Legendre-Gauss-Radau pseudospectral method. The package simplifies the implementation of MPC by allowing users to define system dynamics, constraints, and objectives easily, making it accessible for both research and industrial applications.

This package is introduced in the paper:

**A Practical Open-Source Approach to Model Predictive Control Using the Legendre-Gauss-Radau Pseudospectral Method**  
_Saeid Bayat, James T. Allison_  
[Read the paper here](https://arxiv.org/abs/2310.15960)

## Key Features

- **Open-Source and Free**: Accessible to researchers, educators, and practitioners.
- **Black-Box System Support**: Flexible enough to handle symbolic or black-box models.
- **Simplified Workflow**: Modular design for defining dynamics, constraints, and objectives.
- **Efficient Optimization**: Leverages the pseudospectral method for robust and computationally efficient solutions.
- **Educational Utility**: Useful as a teaching tool for advanced control methods.
- **Broad Applicability**: Suitable for domains like robotics, energy systems, and dynamic optimization.

## Installation Guide

### Prerequisites
Ensure [Conda](https://docs.conda.io/en/latest/) is installed on your system.

### Steps
1. **Clone the Repository**
   ```bash
   git clone https://github.com/saeidb71/LGR_MPC.git
   cd LGR_MPC
   ```

2. **Add Conda-Forge Channel**
   ```bash
   conda config --add channels conda-forge
   ```

3. **Create the Conda Environment**
   ```bash
   conda env create --name MPC-env -f environment.yml
   ```

4. **Activate the Conda Environment**
   ```bash
   conda activate MPC-env
   ```

5. **Install the Package**
   ```bash
   pip install -e .
   ```

---

## References

If you use this package in your research, please cite:

```bibtex
@article{bayat2025practical,
  title={A practical open-source approach to Model Predictive Control using the Legendre--Gauss--Radau pseudospectral method},
  author={Bayat, Saeid and Allison, James T},
  journal={Software Impacts},
  pages={100769},
  year={2025},
  publisher={Elsevier}
}
```

---

## License

This project is licensed under the MIT License. See the LICENSE file for details.

---

## Contact

For questions or support, contact Saeid Bayat at [saeidb@umich.edu](mailto:saeidb@umich.edu).

---
