"""Run the twin loop harness (tests/js/twin_loops.js) as part of the Python suite.

The digital twin closes the leg linkage in the browser: per leg a knee four-bar (cam -> pushrod ->
tibia) and, hanging off it, an achilles four-bar (thigh -> achilles -> foot). The harness loads the
published homing MJCF (static/twin/dash01_homing_v8.xml) with the real twin3d.js and checks that
the homing pose closes all four loops with every passive joint at 0, and that moved motors still
close them.

Skipped where node is not installed; it is not a build dependency of the robot.
"""
import os
import shutil
import subprocess

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
HARNESS = os.path.join(HERE, "js", "twin_loops.js")


@pytest.mark.skipif(shutil.which("node") is None, reason="node is not installed")
def test_the_twin_closes_both_loops_of_each_leg():
    r = subprocess.run([shutil.which("node"), HARNESS], capture_output=True, text=True, timeout=120)
    assert r.returncode == 0, f"twin loop harness failed:\n{r.stdout}\n{r.stderr}"
    assert "all twin loop checks passed" in r.stdout, r.stdout
