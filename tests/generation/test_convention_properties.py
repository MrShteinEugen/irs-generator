"""Property tests for coordinate-frame and attitude convention conversions."""

from __future__ import annotations

import numpy as np
from hypothesis import given, settings
from hypothesis import strategies as st

from irs_generator.generation import (
    AngleUnit,
    Axis,
    InputConvention,
    SignedAxis,
    SignedAxisMapping,
)
from irs_generator.irs_model.rotation import euler_to_dcm_body_to_nav
from irs_generator.navigation_model import EulerAngles

type VectorComponents = tuple[float, float, float]

_AXIS_ORDER = st.permutations((Axis.X, Axis.Y, Axis.Z))
_SIGNS = st.tuples(
    st.sampled_from((-1, 1)),
    st.sampled_from((-1, 1)),
    st.sampled_from((-1, 1)),
)
_FINITE_COMPONENT = st.floats(
    min_value=-1e6,
    max_value=1e6,
    allow_nan=False,
    allow_infinity=False,
)
_VECTOR = st.tuples(_FINITE_COMPONENT, _FINITE_COMPONENT, _FINITE_COMPONENT)
_PITCH_RAD = st.floats(
    min_value=-1.4,
    max_value=1.4,
    allow_nan=False,
    allow_infinity=False,
)
_ANGLE_RAD = st.floats(
    min_value=-np.pi,
    max_value=np.pi,
    allow_nan=False,
    allow_infinity=False,
)


def _mapping(
    axis_order: list[Axis], signs: tuple[int, int, int]
) -> SignedAxisMapping:
    return SignedAxisMapping(
        *(SignedAxis(axis, sign) for axis, sign in zip(axis_order, signs, strict=True))
    )


@settings(max_examples=200)
@given(axis_order=_AXIS_ORDER, signs=_SIGNS, vector=_VECTOR)
def test_signed_axis_mapping_is_invertible(
    axis_order: list[Axis],
    signs: tuple[int, int, int],
    vector: VectorComponents,
) -> None:
    mapping = _mapping(axis_order, signs)

    restored = mapping.inverse().transform_vector(mapping.transform_vector(vector))

    assert np.array_equal(restored, np.asarray(vector, dtype=np.longdouble))


@settings(max_examples=200)
@given(axis_order=_AXIS_ORDER, signs=_SIGNS, vector=_VECTOR)
def test_signed_axis_mapping_preserves_vector_norm_and_orthogonality(
    axis_order: list[Axis],
    signs: tuple[int, int, int],
    vector: VectorComponents,
) -> None:
    mapping = _mapping(axis_order, signs)
    matrix = mapping.as_matrix()
    source = np.asarray(vector, dtype=np.longdouble)
    transformed = mapping.transform_vector(source)

    assert np.array_equal(matrix @ matrix.T, np.eye(3, dtype=np.longdouble))
    assert np.isclose(
        np.dot(transformed, transformed), np.dot(source, source), rtol=1e-15, atol=0.0
    )


@settings(max_examples=200)
@given(
    axis_order=_AXIS_ORDER,
    signs=_SIGNS,
    pitch=_PITCH_RAD,
    roll=_ANGLE_RAD,
    heading=_ANGLE_RAD,
)
def test_input_convention_matches_dcm_frame_change(
    axis_order: list[Axis],
    signs: tuple[int, int, int],
    pitch: float,
    roll: float,
    heading: float,
) -> None:
    mapping = _mapping(axis_order, signs)
    convention = InputConvention(mapping, mapping, AngleUnit.RADIANS)

    converted_attitude = convention.convert_attitude(pitch, roll, heading)
    source_dcm = euler_to_dcm_body_to_nav(EulerAngles(pitch, roll, heading))
    expected_dcm = mapping.as_matrix() @ source_dcm @ mapping.as_matrix().T

    assert np.allclose(
        euler_to_dcm_body_to_nav(converted_attitude), expected_dcm, atol=1e-12
    )


@settings(max_examples=200)
@given(
    axis_order=_AXIS_ORDER,
    signs=_SIGNS,
    pitch=_PITCH_RAD,
    roll=_ANGLE_RAD,
    heading=_ANGLE_RAD,
)
def test_degree_and_radian_input_conventions_are_equivalent(
    axis_order: list[Axis],
    signs: tuple[int, int, int],
    pitch: float,
    roll: float,
    heading: float,
) -> None:
    mapping = _mapping(axis_order, signs)
    radians = InputConvention(mapping, mapping, AngleUnit.RADIANS)
    degrees = InputConvention(mapping, mapping, AngleUnit.DEGREES)

    attitude_in_radians = radians.convert_attitude(pitch, roll, heading)
    attitude_in_degrees = degrees.convert_attitude(
        np.rad2deg(pitch), np.rad2deg(roll), np.rad2deg(heading)
    )

    assert np.allclose(
        euler_to_dcm_body_to_nav(attitude_in_radians),
        euler_to_dcm_body_to_nav(attitude_in_degrees),
        atol=1e-12,
    )
