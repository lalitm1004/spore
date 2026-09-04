import pytest

from robot.hal import Adc, SampledSensors


def test_adc_maps_full_scale_to_the_top_count():
    adc = Adc(bits=10, full_scale=1000.0)

    assert adc.max_count == 1023
    assert adc.counts(1000.0) == 1023
    assert adc.counts(0.0) == 0


def test_adc_clamps_readings_outside_its_range():
    adc = Adc(bits=10, full_scale=1000.0)

    assert adc.counts(1200.0) == 1023
    assert adc.counts(-5.0) == 0


def test_adc_quantises_away_detail_finer_than_one_count():
    # One count is ~0.98 of a raw unit, so these must collapse together.
    adc = Adc(bits=10, full_scale=1000.0)

    assert adc.counts(500.0) == adc.counts(500.4)


def test_sensors_hold_their_value_between_sample_instants():
    sensors = SampledSensors(Adc(), sample_period_s=0.1)

    first = sensors.update(t=0.0, raw=[1000.0])
    held = sensors.update(t=0.05, raw=[0.0])       # too early to resample

    assert first == (1023,)
    assert held == (1023,)


def test_sensors_resample_once_the_period_has_elapsed():
    sensors = SampledSensors(Adc(), sample_period_s=0.1)
    sensors.update(t=0.0, raw=[1000.0])

    assert sensors.update(t=0.1, raw=[0.0]) == (0,)


def test_latency_delays_what_the_firmware_can_see():
    sensors = SampledSensors(Adc(), sample_period_s=0.1, latency_s=0.2)

    sensors.update(t=0.0, raw=[1000.0])
    sensors.update(t=0.1, raw=[0.0])

    # At t=0.2 the freshest sample old enough to have arrived is the t=0.0 one.
    assert sensors.update(t=0.2, raw=[0.0]) == (1023,)
    assert sensors.update(t=0.3, raw=[0.0]) == (0,)
