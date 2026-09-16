# Testing the Published PyPI Package

This directory tests the `irs-generator` distribution published on PyPI.

The setup commands below create a separate virtual environment and install the
published package. The test verifies that imports come from that environment,
not from the working copy. Unpublished changes in this repository are not tested.

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

- The package has distribution metadata and is imported from the virtual environment.
- `SyntheticDataGenerator`, together with `DcmStrapdownINS`, produces two consistent
  steps for a minimal stationary trajectory.
- `DcmTrajectoryReader` and `DcmTrajectoryGenerator` read the prepared CSV file and
  generate `imu.dat` and `gps.dat`.

Installation uses pip's configured package index (PyPI by default). The import
check alone does not establish which index supplied the package.

The test scenarios use public imports only. Temporary input and output files are 
created in the system temporary directory and removed after the test completes.
