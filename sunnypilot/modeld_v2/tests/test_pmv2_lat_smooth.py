import pytest

from cereal import custom
from openpilot.sunnypilot.models.helpers import PMV2_MIN_LAT_SMOOTH, ensure_pmv2_lat_override, get_lat_smooth_seconds


def _make_bundle(internal_name: str, lat: str) -> custom.ModelManagerSP.ModelBundle:
  bundle = custom.ModelManagerSP.ModelBundle()
  bundle.internalName = internal_name
  lat_override = custom.ModelManagerSP.Override()
  lat_override.key = "lat"
  lat_override.value = lat
  bundle.overrides = [lat_override]
  return bundle


class TestGetLatSmoothSeconds:
  def test_pmv2_bumps_zero_to_min(self):
    bundle = _make_bundle("PMV2", ".0")
    assert get_lat_smooth_seconds(bundle) == pytest.approx(PMV2_MIN_LAT_SMOOTH)

  def test_pmv2_keeps_explicit_lat(self):
    bundle = _make_bundle("PMV2", ".2")
    assert get_lat_smooth_seconds(bundle) == pytest.approx(0.2)

  def test_non_pmv2_unchanged(self):
    bundle = _make_bundle("TCPMV3", ".0")
    assert get_lat_smooth_seconds(bundle) == pytest.approx(0.0)


class TestEnsurePmv2LatOverride:
  def test_pmv2_override_applied(self):
    overrides = ensure_pmv2_lat_override({"lat": ".0", "long": ".3"}, "PMV2")
    assert overrides["lat"] == ".15"

  def test_other_models_unchanged(self):
    overrides = ensure_pmv2_lat_override({"lat": ".0"}, "TCPMV3")
    assert overrides["lat"] == ".0"
