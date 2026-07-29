from dataclasses import dataclass
from math import inf, nan, pi

import pytest

from irs_generator.earth_model import (
    EarthModel,
    EllipsoidalEarthModel,
    GRS80EarthModel,
    ReferenceEllipsoid,
    RotationParameters,
    SphericalEarthModel,
    WGS84EarthModel,
)


@dataclass(frozen=True)
class RecordingGravity:
    value_m_s2: float = 7.5

    def gravity_m_s2(self, latitude_rad: float, height_m: float = 0.0) -> float:
        return self.value_m_s2 + latitude_rad + height_m


def test_wgs84_model_has_expected_components_and_reference_gravity() -> None:
    model = WGS84EarthModel()

    assert isinstance(model, EarthModel)
    assert model.name == "WGS 84"
    assert model.ellipsoid.semi_major_axis_m == pytest.approx(6_378_137.0)
    assert model.rotation.angular_velocity_rad_s == pytest.approx(7.292_115e-5)
    assert model.gravity_m_s2(0.0) == pytest.approx(9.780_325_335_9, abs=1e-9)


def test_grs80_model_is_a_distinct_earth_reference_model() -> None:
    wgs84 = WGS84EarthModel()
    grs80 = GRS80EarthModel()

    assert isinstance(grs80, EarthModel)
    assert grs80.name == "GRS 80"
    assert grs80.ellipsoid.inverse_flattening != wgs84.ellipsoid.inverse_flattening
    assert grs80.gravitational_parameter_m3_s2 != wgs84.gravitational_parameter_m3_s2


def test_spherical_model_defaults_to_mean_radius_and_inverse_square_gravity() -> None:
    model = SphericalEarthModel()

    assert isinstance(model, EarthModel)
    assert model.mean_radius_m == pytest.approx(6_371_008.8)
    assert model.meridional_radius_m(0.5) == pytest.approx(model.radius_m)
    assert model.prime_vertical_radius_m(-0.5) == pytest.approx(model.radius_m)
    assert model.gravity_m_s2(0.0) == pytest.approx(
        model.gravitational_parameter_m3_s2 / model.radius_m**2
    )


def test_spherical_model_delegates_to_custom_gravity_model() -> None:
    gravity = RecordingGravity()
    model = SphericalEarthModel(gravity_model=gravity)

    assert model.gravity_model is gravity
    assert model.gravity_m_s2(0.5, 100.0) == pytest.approx(108.0)


def test_ellipsoidal_model_delegates_geometry_and_gravity() -> None:
    ellipsoid = ReferenceEllipsoid(6_378_137.0, 298.257_223_563)
    gravity = RecordingGravity()
    model = EllipsoidalEarthModel(
        name="Custom",
        ellipsoid=ellipsoid,
        rotation=RotationParameters(0.0),
        gravitational_parameter_m3_s2=1.0,
        gravity_model=gravity,
    )

    assert isinstance(model, EarthModel)
    assert model.mean_radius_m == pytest.approx(ellipsoid.mean_radius_m)
    assert model.meridional_radius_m(pi / 4) == pytest.approx(
        ellipsoid.meridional_radius_m(pi / 4)
    )
    assert model.gravity_m_s2(0.5, 100.0) == pytest.approx(108.0)


@pytest.mark.parametrize("name", ["", "   "])
def test_models_reject_empty_name(name: str) -> None:
    with pytest.raises(ValueError):
        SphericalEarthModel(name=name)


@pytest.mark.parametrize("value", [0.0, -1.0, nan, inf, -inf])
def test_spherical_model_rejects_invalid_radius_and_gravitational_parameter(
    value: float,
) -> None:
    with pytest.raises(ValueError):
        SphericalEarthModel(radius_m=value)
    with pytest.raises(ValueError):
        SphericalEarthModel(gravitational_parameter_m3_s2=value)


@pytest.mark.parametrize("latitude_rad", [-pi, pi, nan, inf, -inf])
def test_spherical_model_rejects_invalid_latitudes(latitude_rad: float) -> None:
    model = SphericalEarthModel()
    with pytest.raises(ValueError):
        model.meridional_radius_m(latitude_rad)
    with pytest.raises(ValueError):
        model.prime_vertical_radius_m(latitude_rad)


def test_model_rejects_object_without_gravity_protocol() -> None:
    with pytest.raises(TypeError, match="gravity_model"):
        SphericalEarthModel(gravity_model=object())  # type: ignore[arg-type]
