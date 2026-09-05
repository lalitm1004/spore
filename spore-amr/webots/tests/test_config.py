import pytest

from robot.config import ControllerConfig
from tools.manifest import deep_merge


def test_deep_merge_overrides_nested_keys_without_dropping_siblings():
    base = {"pid": {"kp": 0.9, "ki": 0.0, "kd": 0.04}, "base_speed": 4.0}
    override = {"pid": {"kp": 1.4}}

    assert deep_merge(base, override) == {
        "pid": {"kp": 1.4, "ki": 0.0, "kd": 0.04},
        "base_speed": 4.0,
    }


def test_deep_merge_does_not_mutate_its_inputs():
    base = {"pid": {"kp": 0.9}}
    override = {"pid": {"kp": 1.4}}

    deep_merge(base, override)

    assert base == {"pid": {"kp": 0.9}}
    assert override == {"pid": {"kp": 1.4}}


SENSORS = {"offsets": [0.02, 0.0, -0.02], "white_ref": 1000, "black_ref": 200}


def test_controller_config_reads_the_generated_document():
    config = ControllerConfig.from_dict(
        {
            "name": "bot_01",
            "sensors": SENSORS,
            "control": {"base_speed": 4.0, "pid": {"kp": 0.9, "ki": 0.0, "kd": 0.04}},
        }
    )

    assert config.name == "bot_01"
    assert config.sensors.offsets == (0.02, 0.0, -0.02)
    assert config.control.pid.kp == 0.9
    assert config.control.max_speed == 20.0          # default
    assert config.sensors.min_confidence == 0.15     # default


def test_a_config_whose_white_reference_is_not_above_black_is_rejected():
    broken = dict(SENSORS, white_ref=200, black_ref=200)

    with pytest.raises(ValueError, match="white_ref"):
        ControllerConfig.from_dict({"name": "bot_01", "sensors": broken, "control": {}})


def test_sensor_config_carries_adc_and_sampling_settings():
    config = ControllerConfig.from_dict(
        {
            "name": "bot_01",
            "sensors": {
                "offsets": [0.02, 0.0, -0.02],
                "adc": {"bits": 10, "full_scale": 1000.0},
                "sample_period_s": 0.016,
                "latency_s": 0.0,
            },
            "control": {},
        }
    )

    assert config.sensors.adc.bits == 10
    assert config.sensors.adc.max_count == 1023
    assert config.sensors.sample_period_s == 0.016
    # Calibration references are in ADC counts now, not raw simulator units.
    assert config.sensors.white_ref == 1023
    assert config.sensors.black_ref == 205


def test_start_delay_defaults_to_leaving_immediately():
    config = ControllerConfig.from_dict(
        {"name": "bot_01", "sensors": SENSORS, "control": {}})

    assert config.control.start_delay_s == 0.0


def test_start_delay_is_read_from_the_generated_document():
    config = ControllerConfig.from_dict(
        {"name": "bot_04", "sensors": SENSORS, "control": {"start_delay_s": 12.0}})

    assert config.control.start_delay_s == 12.0
