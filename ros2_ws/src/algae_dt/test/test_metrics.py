"""TDD for lib/metrics.py — latency + CSV-row formatting (PLAN T3.1, Rubric ②).

The CSV is the trustworthy evidence for the review, so its formatting is unit-pinned: stable
columns, NaN/inf handled, booleans as 0/1, and the optional safety-event stop_skew_ms blank on
ordinary ticks.
"""
from algae_dt.lib import metrics as M

INF = float('inf')


def test_latency_ms_sign_and_value():
    assert M.latency_ms(1.0, 1.25) == 250.0
    assert M.latency_ms(2.0, 2.0) == 0.0


def test_csv_header_columns():
    cols = M.csv_header().split(',')
    assert cols == ['stamp_s', 'dxy_m', 'dyaw_rad', 'sensor_err_m',
                    'latency_ms', 'in_tolerance', 'stop_skew_ms']


def test_csv_row_basic_formatting():
    row = M.csv_row(12.5, 0.1234, 0.05, 0.2, 123.4, True)
    fields = row.split(',')
    assert len(fields) == 7
    assert fields[0] == '12.5000'
    assert fields[5] == '1'           # in_tolerance True -> 1
    assert fields[6] == ''            # stop_skew omitted -> blank


def test_csv_row_in_tolerance_false_is_zero():
    assert M.csv_row(0.0, 0.0, 0.0, 0.0, 0.0, False).split(',')[5] == '0'


def test_csv_row_includes_stop_skew_when_given():
    assert M.csv_row(0.0, 0.0, 0.0, 0.0, 0.0, True, stop_skew_ms=42.0).split(',')[6] == '42.0'


def test_csv_row_handles_inf_sensor_error():
    # one world sees an obstacle the other doesn't -> sensor_err is inf; must not crash.
    assert M.csv_row(0.0, 0.0, 0.0, INF, 0.0, False).split(',')[3] == 'inf'


def test_csv_row_round_trips_through_header_width():
    assert len(M.csv_row(1, 2, 3, 4, 5, True, 6).split(',')) == len(M.csv_header().split(','))
