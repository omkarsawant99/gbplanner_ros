"""Run the C++ candidate-ordering regression with the package's pytest suite."""
from pathlib import Path
import shutil
import subprocess
import pytest


def test_candidate_search(tmp_path):
    compiler = shutil.which('c++')
    if compiler is None:
        pytest.skip('C++ compiler required for planner candidate search test')
    package = Path(__file__).resolve().parents[1]
    executable = tmp_path / 'test_waypoint_candidates'
    subprocess.run([compiler, '-std=c++17', '-I', str(package / 'include'),
                    str(package / 'test/test_waypoint_candidates.cpp'),
                    '-o', str(executable)], check=True, capture_output=True)
    subprocess.run([str(executable)], check=True, capture_output=True)
