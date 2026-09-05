"""`docs/scenarios.md` must stay true.

A behavioural contract that quietly stops matching the code is worse than no
contract: it reads as authoritative and is wrong. So the ids in the doc and the
tests that prove them are checked against each other here.

In-process and instant -- this is about two files agreeing, not about the fleet.
"""
from __future__ import annotations

import re
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
DOC = ROOT / "docs" / "scenarios.md"
DOCKER_TESTS = ROOT / "tests" / "test_docker.py"

#: Ids look like A1, B12, D1_D2_D3 -- a letter and a number, in a table cell.
_DOC_ID = re.compile(r"^\|\s*([A-H]\d+)\s*\|", re.M)
_TEST_ID = re.compile(r"^def test_([A-H]\d+(?:_[A-H]\d+)*)_", re.M)


def _doc_ids() -> set[str]:
    return set(_DOC_ID.findall(DOC.read_text()))


def _test_ids() -> set[str]:
    ids: set[str] = set()
    for name in _TEST_ID.findall(DOCKER_TESTS.read_text()):
        # `D1_D2_D3` is one test covering three rows of the table.
        ids.update(name.split("_"))
    return ids


def test_the_contract_doc_exists():
    assert DOC.is_file(), "docs/scenarios.md is the fleet's behavioural contract"


def test_every_documented_scenario_has_a_test():
    """A row in the doc with no test behind it is a claim nobody checks."""
    documented, tested = _doc_ids(), _test_ids()
    # Scenarios the doc explicitly lists as not covered, with its reasons, are
    # named in the "What is not covered" section rather than in a table row.
    missing = sorted(documented - tested)
    assert not missing, (
        f"documented but untested: {missing}. Either write the test or move the "
        "row into 'What is not covered' with a reason."
    )


def test_every_scenario_test_is_documented():
    """And the reverse: a test with no row is behaviour nobody wrote down."""
    undocumented = sorted(_test_ids() - _doc_ids())
    assert not undocumented, f"tested but undocumented: {undocumented}"


def test_the_doc_is_honest_about_its_gaps():
    """The gaps section is the part most likely to rot, because it is the part
    that stops being true when someone closes a gap."""
    text = DOC.read_text()
    assert "What is not covered" in text
    # Closing a gap means editing this list, on purpose. "synthetic" was here
    # while obstructions were pushed in through an admin RPC; a robot reports
    # them now, so the word is gone and so is the entry. That is the guard
    # working -- it made someone come here and say the gap had been closed.
    for known_gap in ("camera", "F1", "compressed"):
        assert known_gap in text, f"the doc no longer mentions {known_gap!r}"


@pytest.mark.parametrize("guarantee", [
    "never answered with silence",
    "No two robots ever hold one node",
    "same verdict alone",
    "survives the leader",
])
def test_the_four_guarantees_are_stated(guarantee):
    """These are the promises the rest of the document is detail about."""
    assert guarantee in DOC.read_text()
