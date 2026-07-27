from math import inf, nan, pi

import pytest

from irs_generator.earth_model import ReferenceEllipsoid


@pytest.fixture
def wgs84_ellipsoid() -> ReferenceEllipsoid:
    return ReferenceEllipsoid(
        semi_major_axis_m=6_378_137.0,
        inverse_flattening=298.257_223_563,
    )


def test_ellipsoid_derives_wgs84_geometric_parameters(
    wgs84_ellipsoid: ReferenceEllipsoid,
) -> None:
    assert wgs84_ellipsoid.flattening == pytest.approx(1 / 298.257_223_563)
    assert wgs84_ellipsoid.semi_minor_axis_m == pytest.approx(6_356_752.314_245_179)
    assert wgs84_ellipsoid.first_eccentricity_squared == pytest.approx(
        0.006_694_379_990_141_316_5
    )
    assert wgs84_ellipsoid.second_eccentricity_squared == pytest.approx(
        0.006_739_496_742_276_434
    )


def test_radius_properties_match_their_definitions(
    wgs84_ellipsoid: ReferenceEllipsoid,
) -> None:
    a = wgs84_ellipsoid.semi_major_axis_m
    b = wgs84_ellipsoid.semi_minor_axis_m

    assert wgs84_ellipsoid.mean_radius_m == pytest.approx((2 * a + b) / 3)
    assert wgs84_ellipsoid.volumetric_radius_m == pytest.approx((a * a * b) ** (1 / 3))


def test_curvature_radii_have_reference_values_at_equator_and_pole(
    wgs84_ellipsoid: ReferenceEllipsoid,
) -> None:
    assert wgs84_ellipsoid.meridional_radius_m(0.0) == pytest.approx(
        6_335_439.327_292_819
    )
    assert wgs84_ellipsoid.prime_vertical_radius_m(0.0) == pytest.approx(
        6_378_137.0
    )
    assert wgs84_ellipsoid.meridional_radius_m(pi / 2) == pytest.approx(
        6_399_593.625_758_493
    )
    assert wgs84_ellipsoid.prime_vertical_radius_m(pi / 2) == pytest.approx(
        6_399_593.625_758_493
    )


def test_geocentric_surface_radius_equals_axes_at_equator_and_pole(
    wgs84_ellipsoid: ReferenceEllipsoid,
) -> None:
    assert wgs84_ellipsoid.geocentric_surface_radius_m(0.0) == pytest.approx(
        wgs84_ellipsoid.semi_major_axis_m
    )
    assert wgs84_ellipsoid.geocentric_surface_radius_m(pi / 2) == pytest.approx(
        wgs84_ellipsoid.semi_minor_axis_m
    )


@pytest.mark.parametrize(
    "method_name",
    [
        "meridional_radius_m",
        "prime_vertical_radius_m",
        "geocentric_surface_radius_m",
    ],
)
def test_geometric_radii_are_symmetric_about_equator(
    wgs84_ellipsoid: ReferenceEllipsoid,
    method_name: str,
) -> None:
    method = getattr(wgs84_ellipsoid, method_name)
    assert method(0.7) == pytest.approx(method(-0.7))


@pytest.mark.parametrize("semi_major_axis_m", [0.0, -1.0, nan, inf, -inf])
def test_ellipsoid_rejects_invalid_semi_major_axis(semi_major_axis_m: float) -> None:
    with pytest.raises(ValueError):
        ReferenceEllipsoid(semi_major_axis_m, 298.257_223_563)


@pytest.mark.parametrize("inverse_flattening", [1.0, 0.0, -1.0, nan, inf, -inf])
def test_ellipsoid_rejects_invalid_inverse_flattening(
    inverse_flattening: float,
) -> None:
    with pytest.raises(ValueError):
        ReferenceEllipsoid(6_378_137.0, inverse_flattening)


@pytest.mark.parametrize("latitude_rad", [-pi, pi, nan, inf, -inf])
def test_geometric_radii_reject_invalid_latitude(
    wgs84_ellipsoid: ReferenceEllipsoid,
    latitude_rad: float,
) -> None:
    with pytest.raises(ValueError):
        wgs84_ellipsoid.meridional_radius_m(latitude_rad)
