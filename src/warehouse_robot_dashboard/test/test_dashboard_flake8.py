# Copyright 2026 Tanishk Patidar
#
# This source code is provided for viewing, evaluation, educational,
# and portfolio purposes. All rights reserved.
#
# See the repository LICENSE file for terms governing copying,
# modification, distribution, and commercial use.
#
from pathlib import Path

from ament_flake8.main import main_with_errors
import pytest


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    package_root = Path(__file__).resolve().parents[1]
    source_dirs = [
        package_root / 'warehouse_robot_dashboard',
        package_root / 'test',
    ]
    rc, errors = main_with_errors(
        argv=[str(path) for path in source_dirs]
    )
    assert rc == 0, 'Found %d code style errors / warnings:\n' % len(errors) + '\n'.join(errors)
