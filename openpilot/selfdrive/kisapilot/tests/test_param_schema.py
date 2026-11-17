"""Check the settings/API contract without device or native Params bindings."""
import json
from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[4]


class TestParamSchema(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    cls.schema = json.loads((ROOT / 'openpilot/selfdrive/kisapilot/param_schema.json').read_text(encoding='utf-8'))
    registry = (ROOT / 'openpilot/common/params_keys.h').read_text(encoding='utf-8')
    cls.registry = {
      key: (kind, default)
      for key, kind, default in re.findall(
        r'\{"(\w+)",\s*\{[^,\n]+,\s*(\w+)(?:,\s*"([^"\n]*)")?\}\}', registry)
    }

  def test_unique_registered_keys_and_types(self):
    keys = [entry['param'] for entry in self.schema]
    self.assertEqual(len(keys), len(set(keys)))
    for entry in self.schema:
      with self.subTest(param=entry['param']):
        kind, _ = self.registry[entry['param']]
        self.assertEqual(entry['value_type'], kind)

  def test_numeric_ranges_and_defaults(self):
    for entry in self.schema:
      with self.subTest(param=entry['param']):
        kind, default = self.registry[entry['param']]
        if kind == 'BOOL':
          self.assertIn(default, ('0', '1'))
          continue
        low, high, step = (entry[key] for key in ('min_value', 'max_value', 'step'))
        self.assertLess(low, high)
        self.assertGreater(step, 0)
        self.assertGreaterEqual(float(default), low)
        self.assertLessEqual(float(default), high)
        self.assertGreaterEqual(entry['decimal_places'], 0)
        if kind == 'INT':
          self.assertEqual(step, int(step))
          self.assertEqual(entry['decimal_places'], 0)

  def test_bilingual_text_and_special_value_bounds(self):
    for entry in self.schema:
      with self.subTest(param=entry['param']):
        for field in ('title', 'description'):
          for language in ('ko', 'en'):
            self.assertTrue(entry[field][language].strip())
        for value, label in entry.get('special_texts', {}).items():
          self.assertGreaterEqual(float(value), entry['min_value'])
          self.assertLessEqual(float(value), entry['max_value'])
          for language in ('ko', 'en'):
            self.assertTrue(label[language].strip())


if __name__ == '__main__':
  unittest.main()
