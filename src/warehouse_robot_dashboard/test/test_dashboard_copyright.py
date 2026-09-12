from pathlib import Path

import pytest


@pytest.mark.linter
def test_copyright():
    package_root = Path(__file__).resolve().parents[1]

    files = [
        path for path in package_root.rglob('*.py')
        if '__pycache__' not in path.parts
    ]

    for path in files:
        text = path.read_text()
        assert 'Copyright ' in text, (
            f'Missing copyright notice: {path}'
        )
