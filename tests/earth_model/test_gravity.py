from math import inf, nan, pi

import pytest

from irs_generator.earth_model.geometry import ReferenceEllipsoid
from irs_generator.earth_model.gravity import (
    ConstantGravity,
    GravityModel,
    InverseSquareGravity,
    SomiglianaNormalGravity,
)
from irs_generator.earth_model.rotation import RotationParamet

@pytest.fixture
def wgs84_normal_gravity() -> SomiglianaNormalGravity:
    return SomiglianaNormalGravity(
        ellipsoid=ReferenceEllipsoid(6_378_137.0, 298.257_223_563),
        rotation=RotationParameters(7.292_115e-5),
        gravitational_parameter_m3_s2=3.986_004_418e14,
    )


def test_gravity_implementations_satisfy_runtime_protocol(
    wgs84_normal_gravity: SomiglianaNormalGravity,
) -> None:
    assert isinstance(wgs84_normal_gravity, GravityModel)
    assert isinstance(InverseSquareGravity(100.0, 10.0), GravityModel)
    assert isinstance(ConstantGravity(), GravityModel)


def test_somigliana_surface_gravity_matches_wgs84_reference_values(
    wgs84_normal_gravity: SomiglianaNormalGravity,
) -> None:
    assert wgs84_normal_gravity.surface_gravity_m_s2(0.0) == pytest.approx(
        9.780_325_335_9,
        abs=1e-9,
    )
    assert wgs84_normal_gravity.surface_gravity_m_s2(pi / 2) == pytest.approx(
        9.832_184_937_9,
        abs=1e-9,
    )


@pytest.mark.parametrize("latitude_rad", [0.0, 0.8, -0.8, pi / 2])
def test_somigliana_zero_height_equals_surface_gravity(
    wgs84_normal_gravity: SomiglianaNormalGravity,
    latitude_rad: float,
) -> None:
    assert wgs84_normal_gravity.gravity_m_s2(latitude_rad) == pytest.approx(
        wgs84_normal_gravity.surface_gravity_m_s2(latitude_rad)
    )


def test_somigliana_gravity_decreases_above_ellipsoid(
    wgs84_normal_gravity: SomiglianaNormalGravity,
) -> None:
    assert wgs84_normal_gravity.gravity_m_s2(0.5, 1_000.0) < (
        wgs84_normal_gravity.gravity_m_s2(0.5)
    )


@pytest.mark.parametrize("latitude_rad", [-pi, pi, nan, inf, -inf])
def test_somigliana_rejects_invalid_latitude(
    wgs84_normal_gravity: SomiglianaNormalGravity,
    latitude_rad: float,
) -> None:
    with pytest.raises(ValueError):
        wgs84_normal_gravity.gravity_m_s2(latitude_rad)


@pytest.mark.parametrize("height_m", [nan, inf, -inf, -6_378_137.0])
def test_somigliana_rejects_invalid_or_central_height(
    wgs84_normal_gravity: SomiglianaNormalGravity,
    height_m: float,
) -> None:
    with pytest.raises(ValueError):
        wgs84_normal_gravity.gravity_m_s2(0.0, height_m)


def test_inverse_square_gravity_follows_inverse_square_law() -> None:
    gravity = InverseSquareGravity(
        gravitational_parameter_m3_s2=100.0,
        reference_radius_m=10.0,
    )

    assert gravity.gravity_m_s2(0.0) == pytest.approx(1.0)
    assert gravity.gravity_m_s2(0.0, height_m=10.0) == pytest.approx(0.25)


@pytest.mark.parametrize("value", [0.0, -1.0, nan, inf, -inf])
def test_inverse_square_gravity_rejects_invalid_constructor_values(value: float) -> None:
    with pytest.raises(ValueError):
        InverseSquareGravity(value, 10.0)
    with pytest.raises(ValueError):
        InverseSquareGravity(100.0, value)


@pytest.mark.parametrize("height_m", [-10.0, -11.0, nan, inf, -inf])
def test_inverse_square_gravity_rejects_invalid_radius_or_height(height_m: float) -> None:
    gravity = InverseSquareGravity(100.0, 10.0)
    with pytest.raises(ValueError):
        gravity.gravity_m_s2(0.0, height_m)


def test_constant_gravity_is_independent_of_position() -> None:
    gravity = ConstantGravity(9.81)
    assert gravity.gravity_m_s2(-0.5, -100.0) == pytest.approx(9.81)
    assert gravity.gravity_m_s2(0.5, 100_000.0) == pytest.approx(9.81)


@pytest.mark.parametrize("value", [0.0, -1.0, nan, inf, -inf])
def test_constant_gravity_rejects_invalid_value(value: float) -> None:
    with pytest.raises(ValueError):
        ConstantGravity(value)
