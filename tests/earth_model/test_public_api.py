import irs_generator.earth_model as earth_model
from irs_generator.earth_model import geometry, gravity, models, rotation


def test_package_exports_every_declared_public_name() -> None:
    for name in earth_model.__all__:
        assert hasattr(earth_model, name)


def test_module_exports_match_canonical_owners() -> None:
    assert geometry.__all__ == ["ReferenceEllipsoid"]
    assert rotation.__all__ == ["RotationParameters"]
    assert gravity.__all__ == [
        "ConstantGravity",
        "GravityModel",
        "InverseSquareGravity",
        "SomiglianaNormalGravity",
    ]
    assert models.__all__ == [
        "EarthModel",
        "EllipsoidalEarthModel",
        "GRS80EarthModel",
        "SphericalEarthModel",
        "WGS84EarthModel",
    ]


def test_package_exports_are_the_same_objects_as_canonical_modules() -> None:
    assert earth_model.ReferenceEllipsoid is geometry.ReferenceEllipsoid
    assert earth_model.RotationParameters is rotation.RotationParameters
    assert earth_model.ConstantGravity is gravity.ConstantGravity
    assert earth_model.WGS84EarthModel is models.WGS84EarthModel
