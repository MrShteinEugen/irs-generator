# Testing the Published PyPI Package

This directory tests the `irs-generator` distribution published on PyPI.

The test does not use the source code from this repository. Instead, it creates a separate 
virtual environment, installs the package into it, and verifies the import path.

Python 3.12 or later is required.

## Quick Start in PowerShell

Run the following commands from this directory:

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install --upgrade pip
.\.venv\Scripts\python.exe -m pip install --no-cache-dir -r requirements.txt
.\.venv\Scripts\python.exe -I .\test_pypi_install.py
```

The `-I` option isolates Python from user-specific import paths. Therefore, a successful 
run confirms that the test is using the package installed in `.venv`, rather than the 
package from the working copy.

## What Is Tested

* The package is installed from PyPI, has distribution metadata, and is imported from 
* the newly created virtual environment.
* `SyntheticDataGenerator`, together with `DcmStrapdownINS`, produces two consistent 
* steps for a minimal stationary trajectory.
* `DcmTrajectoryReader` and `DcmTrajectoryGenerator` read the prepared CSV file and 
* generate `imu.dat` and `gps.dat`.

The test scenarios use public imports only. Temporary input and output files are 
created in the system temporary directory and removed after the test completes.
