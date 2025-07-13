from cam_slicer.utils.custom_strategy import load_strategy, run_strategy, Strategy


def test_load_and_run_strategy(tmp_path):
    """Test loading custom strategy and applying it."""
    script = tmp_path / "demo.py"
    script.write_text(
        """
from cam_slicer.utils.custom_strategy import Strategy

def register():
    def apply(tp, params):
        return tp[::-1]
    return Strategy(name='rev', description='Reverse', apply=apply)
"""
    )
    strat = load_strategy(script)
    assert strat.name == 'rev'
    result = run_strategy([1, 2, 3], strat)
    assert result == [3, 2, 1]
