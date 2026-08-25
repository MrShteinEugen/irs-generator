"""Regression tests for the stable package-level public API."""

from importlib.metadata import metadata, version
from importlib.resources import files

import irs_generator
import irs_generator.earth_model as earth_model
import irs_generator.generation as generation
import irs_generator.gps_model as gps_model
import irs_generator.irs_model as irs_model
import irs_generator.navigation_model as navigation_model


def test_root_package_exports_are_intentional() -> None:
    assert irs_generator.__all__ == ["__version__"]


def test_package_version_uses_the_distribution_metadata() -> None:
    assert irs_generator.__version__ == version("irs-generator")


def test_distribution_metadata_uses_the_project_description_and_readme() -> None:
    distribution_metadata = metadata("irs-generator")

    assert distribution_metadata["Summary"] == (
        "Synthesize ideal IMU and GNSS measurements from a target trajectory."
    )
    assert "IRS Generator" in distribution_metadata.get_payload()


def test_layer_package_exports_are_intentional() -> None:
    assert earth_model.__all__ == [
        "ConstantGravity",
        "EarthModel",
        "EllipsoidalEarthModel",
        "GeodeticPosition",
        "GravityModel",
        "GRS80EarthModel",
        "InverseSquareGravity",
        "ReferenceEllipsoid",
        "RotationParameters",
        "SomiglianaNormalGravity",
        "SphericalEarthModel",
        "WGS84EarthModel",
    ]
    assert generation.__all__ == [
        "AngleUnit",
        "Axis",
        "CsvOutputWriter",
        "CsvOutputFormat",
        "CsvTrajectoryReader",
        "CsvTrajectorySchema",
        "DatOutputFormat",
        "DcmTrajectoryGenerator",
        "DcmTrajectoryPoint",
        "DcmTrajectoryReader",
        "GeneratedStep",
        "GenerationConfig",
        "GenerationConvergenceError",
        "GenerationDiagnostics",
        "GenerationError",
        "GenerationMetadata",
        "GenerationSolverError",
        "Handedness",
        "InputConvention",
        "InvalidTrajectoryError",
        "SignedAxis",
        "SignedAxisMapping",
        "TargetTrajectoryPoint",
        "Trajectory",
        "TrajectoryProviderAdapter",
        "TrajectoryPoint",
        "TrajectoryUnits",
        "TrajectoryValidationConfig",
        "TrajectoryValidator",
        "SyntheticDataGenerator",
        "StepSolverConfig",
    ]
    assert gps_model.__all__ == ["GnssSample"]
    assert navigation_model.__all__ == [
        "EulerAngles",
        "NavigationState",
        "NavigationVelocity",
    ]
    assert irs_model.__all__ == [
        "BiasImuErrorModel",
        "CompositeImuErrorModel",
        "CompositeCorrection",
        "CorrectionStrategy",
        "DcmStrapdownINS",
        "AnalyticAlignment",
        "HeightAidingConfig",
        "HeightAidingCorrection",
        "IdealImuErrorModel",
        "ImuErrorModel",
        "ImuSample",
        "InertialReferenceSystem",
        "InertialNavigationAlgorithm",
        "MechanizationConfig",
        "NavigationAlgorithm",
        "NoCorrection",
        "PositionAidingConfig",
        "PositionAidingCorrection",
        "RadialAttitudeConfig",
        "RadialAttitudeCorrection",
        "StrapdownINS",
        "VelocityAidingConfig",
        "VelocityAidingCorrection",
    ]


def test_declared_public_exports_exist() -> None:
    for package in (
        irs_generator,
        earth_model,
        generation,
        gps_model,
        irs_model,
        navigation_model,
    ):
        for name in package.__all__:
            assert hasattr(package, name)


def test_package_declares_pep_561_typing_support() -> None:
    assert files(irs_generator).joinpath("py.typed").is_file()
