import irs_generator.generation as generation
from irs_generator.generation import dcm, formats


def test_package_exports_every_declared_public_name() -> None:
    for name in generation.__all__:
        assert hasattr(generation, name)


def test_dcm_generation_api_uses_canonical_public_names() -> None:
    assert generation.DcmTrajectoryGenerator is dcm.DcmTrajectoryGenerator
    assert generation.DcmTrajectoryPoint is dcm.DcmTrajectoryPoint
    assert generation.DcmTrajectoryReader is dcm.DcmTrajectoryReader
    assert generation.DatOutputFormat is formats.DatOutputFormat


def test_generation_api_does_not_reexport_preview_compatibility_names() -> None:
    assert "LegacyDatFormat" not in generation.__all__
    assert "LegacyDcmReferenceGenerator" not in generation.__all__
    assert "LegacyTrajectoryPoint" not in generation.__all__
    assert "LegacyTrajectoryReader" not in generation.__all__
