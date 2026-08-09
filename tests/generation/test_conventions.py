from itertools import permutations, product

import numpy as np
import pytest

from irs_generator.generation import Axis, Handedness, SignedAxis, SignedAxisMapping


def test_mapping_applies_axis_permutation_and_signs() -> None:
    mapping = SignedAxisMapping(
        SignedAxis(Axis.Y),
        SignedAxis(Axis.X, -1),
        SignedAxis(Axis.Z),
    )

    assert mapping.transform_vector((2.0, 3.0, 5.0)) == pytest.approx(
        (3.0, -2.0, 5.0)
    )


@pytest.mark.parametrize("axes", permutations(Axis))
@pytest.mark.parametrize("signs", product((-1, 1), repeat=3))
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
