# Copyright 2015 Open Source Robotics Foundation, Inc.
#
# This source code is provided for viewing, evaluation, educational,
# and portfolio purposes. All rights reserved.
#
# See the repository LICENSE file for terms governing copying,
# modification, distribution, and commercial use.

from pathlib import Path

from ament_pep257.main import main
import pytest


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    package_root = Path(__file__).resolve().parents[1]
    rc = main(argv=[str(package_root), str(package_root / 'test')])
    assert rc == 0, 'Found code style errors / warnings'
