import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path


class TestStationaryHyundaiBlinker(unittest.TestCase):
  @unittest.skipUnless(shutil.which("cc"), "native C compiler required")
  def test_native_safety_scenarios(self):
    source = Path(__file__).with_name("hyundai_blinker_test.c")
    with tempfile.TemporaryDirectory() as work:
      executable = str(Path(work) / "blinker-test")
      subprocess.run(["cc", "-std=gnu11", "-Wall", "-Wextra", "-Werror", "-DALLOW_DEBUG",
                      "-I", str(source.parents[3]), str(source), "-o", executable], check=True)
      subprocess.run([executable], check=True)
