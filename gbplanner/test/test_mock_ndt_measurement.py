import sys
from pathlib import Path
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from mock_ndt_measurement import MeasurementJobs


def test_completion_after_ten_seconds_not_after_acceptance():
    jobs = MeasurementJobs(10)
    assert jobs.start(50)
    assert not jobs.due(59.99)
    assert jobs.due(60)


def test_duplicate_start_does_not_reset_active_timer():
    jobs = MeasurementJobs(10)
    jobs.start(0)
    jobs.start(9)
    assert jobs.due(10)


def test_next_measurement_and_retry_until_completion_delivered():
    jobs = MeasurementJobs(10)
    jobs.start(0)
    assert jobs.due(11)
    assert jobs.due(12)
    jobs.finish()
    assert not jobs.due(12)
    jobs.start(20)
    assert not jobs.due(29)
    assert jobs.due(30)


def test_invalid_duration():
    for duration in (0, -1, float('nan'), float('inf')):
        with pytest.raises(ValueError):
            MeasurementJobs(duration)
