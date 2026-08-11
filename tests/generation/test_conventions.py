from itertools import permutations, product

import numpy as np
import pytest

from irs_generator.generation import (
    AngleUnit,
    Axis,
    Handedness,
    InputConvention,
    SignedAxis,
    SignedAxisMapping,
)
from irs_generator.irs_model.rotation import euler_to_dcm_body_to_nav
from irs_generator.navigation_model import EulerAngles


def test_mapping_applies_axis_permutation_and_signs() -> None:
    mapping = SignedAxisMapping(
        SignedAxis(Axis.Y),
        SignedAxis(Axis.X, -1),
        SignedAxis(Axis.Z),
    )

    assert mapping.transform_vector((2.0, 3.0, 5.0)) == pytest.approx(
        (3.0, -2.0, 5.0)
    )


@pytest.mark.parametrize("axes", tuple(permutations(Axis)))
@pytest.mark.parametrize("signs", tuple(product((-1, 1), repeat=3)))
def test_every_signed_permutation_has_an_exact_inverse(
    axes: tuple[Axis, Axis, Axis], signs: tuple[int, int, int]
) -> None:
    mapping = SignedAxisMapping(
        *(SignedAxis(axis, sign) for axis, sign in zip(axes, signs, strict=True))
    )
    vector = np.array((2.0, -3.0, 5.0), dtype=np.longdouble)

    assert mapping.inverse().transform_vector(mapping.transform_vector(vector)) == (
        pytest.approx(vector)
    )


def test_identity_mapping_is_right_handed() -> None:
    mapping = SignedAxisMapping.identity()

    assert mapping.handedness is Handedness.RIGHT_HANDED
    mapping.validate_handedness(Handedness.RIGHT_HANDED)


def test_single_axis_reflection_is_left_handed() -> None:
    mapping = SignedAxisMapping(
        SignedAxis(Axis.X, -1), SignedAxis(Axis.Y), SignedAxis(Axis.Z)
    )

    assert mapping.handedness is Handedness.LEFT_HANDED
    with pytest.raises(ValueError, match="expected right_handed mapping"):
        mapping.validate_handedness(Handedness.RIGHT_HANDED)


def test_mapping_rejects_duplicate_source_axes() -> None:
    with pytest.raises(ValueError, match="each source axis"):
        SignedAxisMapping(
            SignedAxis(Axis.X), SignedAxis(Axis.X), SignedAxis(Axis.Z)
        )


@pytest.mark.parametrize("sign", (0, 2, -2, True))
def test_signed_axis_rejects_invalid_sign(sign: int) -> None:
    with pytest.raises(ValueError, match="sign must be -1 or 1"):
        SignedAxis(Axis.X, sign)


def test_canonical_input_convention_preserves_velocity_and_attitude() -> None:
    convention = InputConvention.canonical()

    velocity = convention.convert_navigation_velocity((2.0, -3.0, 5.0))
    attitude = convention.convert_attitude(0.2, -0.3, 0.4)

    assert velocity.as_array() == pytest.approx((2.0, -3.0, 5.0))
    assert attitude.as_array() == pytest.approx((0.2, -0.3, 0.4))


def test_input_convention_converts_navigation_velocity_to_enu() -> None:
    north_east_down_to_enu = SignedAxisMapping(
        SignedAxis(Axis.Y), SignedAxis(Axis.X), SignedAxis(Axis.Z, -1)
    )
    convention = InputConvention(
        navigation_axes=north_east_down_to_enu,
        body_axes=north_east_down_to_enu,
    )

    velocity = convention.convert_navigation_velocity((2.0, 3.0, 5.0))

    assert velocity.as_array() == pytest.approx((3.0, 2.0, -5.0))


def test_input_convention_converts_degrees_and_reference_frames() -> None:
    north_east_down_to_enu = SignedAxisMapping(
        SignedAxis(Axis.Y), SignedAxis(Axis.X), SignedAxis(Axis.Z, -1)
    )
    convention = InputConvention(
        navigation_axes=north_east_down_to_enu,
        body_axes=north_east_down_to_enu,
        angle_unit=AngleUnit.DEGREES,
    )

    attitude = convention.convert_attitude(10.0, -20.0, 30.0)
    expected_dcm = (
        north_east_down_to_enu.as_matrix()
        @ euler_to_dcm_body_to_nav(
            EulerAngles(
                np.deg2rad(10.0), np.deg2rad(-20.0), np.deg2rad(30.0)
            )
        )
        @ north_east_down_to_enu.as_matrix().T
    )

    assert np.allclose(euler_to_dcm_body_to_nav(attitude), expected_dcm)


def test_input_convention_rejects_mismatched_frame_handedness() -> None:
    reflected = SignedAxisMapping(
        SignedAxis(Axis.X, -1), SignedAxis(Axis.Y), SignedAxis(Axis.Z)
    )

    with pytest.raises(ValueError, match="must have the same handedness"):
        InputConvention(SignedAxisMapping.identity(), reflected)
