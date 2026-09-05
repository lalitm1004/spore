"""The journal is durable: state and commands survive a restart via replay."""

from temp_network_interface import Fleet, Journal, NetworkToRobot

from .test_messages import status


def test_journal_appends_and_reads_back(tmp_path):
    journal = Journal(tmp_path / "fleet.jsonl").open()
    try:
        journal.append({"type": "command", "bot_id": 1, "command": {}})
        assert list(journal.read()) == [{"type": "command", "bot_id": 1, "command": {}}]
    finally:
        journal.close()


def test_replay_reconstructs_status_and_pending(tmp_path):
    journal = Journal(tmp_path / "fleet.jsonl").open()
    journal.append({"type": "status", "bot_id": 5,
                    "status": status(bot_id=5, latest_node_id=10).to_dict()})
    journal.append({"type": "command", "bot_id": 5,
                    "command": NetworkToRobot(target_node_id=99, timestamp=1).to_dict()})

    fleet = Fleet.load(journal)
    assert len(fleet) == 1
    assert fleet.robot(5).latest_node_id == 10
    assert [c.target_node_id for c in fleet.pending(5)] == [99]
    journal.close()


def test_replay_reconciles_in_order(tmp_path):
    """A status that follows a command reconciles it during replay, exactly as
    it would live, because reconciliation is deterministic on the log."""
    journal = Journal(tmp_path / "fleet.jsonl").open()
    journal.append({"type": "command", "bot_id": 5,
                    "command": NetworkToRobot(target_node_id=99, timestamp=1).to_dict()})
    journal.append({"type": "status", "bot_id": 5,
                    "status": status(bot_id=5, latest_node_id=99).to_dict()})

    fleet = Fleet.load(journal)
    assert fleet.pending(5) == []
    journal.close()


def test_trailing_partial_line_is_skipped(tmp_path):
    journal = Journal(tmp_path / "fleet.jsonl").open()
    journal.append({"type": "status", "bot_id": 5, "status": status(bot_id=5).to_dict()})
    journal.close()

    # Simulate a crash that truncated the last line.
    with open(tmp_path / "fleet.jsonl", "a") as handle:
        handle.write('{"type": "command", "bot_id": 5, "com')

    journal = Journal(tmp_path / "fleet.jsonl").open()
    fleet = Fleet.load(journal)
    assert len(fleet) == 1
    journal.close()
