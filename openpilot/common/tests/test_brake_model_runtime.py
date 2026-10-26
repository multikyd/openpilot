import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import zipfile

PATH = Path(__file__).resolve().parents[1]/'brake_model_runtime.py'
spec = importlib.util.spec_from_file_location('runtime_under_test',PATH)
runtime = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runtime)


class RuntimePreparationTest(unittest.TestCase):
  def setUp(self):
    self.temp = tempfile.TemporaryDirectory()
    self.addCleanup(self.temp.cleanup)
    self.root = Path(self.temp.name)
    self.bundle = self.root/'bundle'
    self.bundle.mkdir()
    self.base = self.root/'install'
    self.make_bundle()

  def make_bundle(self, bad_path=False):
    wheel = self.bundle/'runtime-1-py3-none-any.whl'
    with zipfile.ZipFile(wheel,'w') as archive:
      archive.writestr('onnxruntime/__init__.py','version=1')
      archive.writestr('onnxruntime/capi/pybind.so',b'fake-fixture')
      archive.writestr('onnxruntime/capi/providers.so',b'fake-fixture')
      if bad_path:
        archive.writestr('../escaped.py','unsafe')
    self.manifest = dict(version='1',system='Linux',machine='aarch64',python='3.12',
                         wheels=[dict(file=wheel.name,size=wheel.stat().st_size,
                                      sha256=hashlib.sha256(wheel.read_bytes()).hexdigest())])
    self.write_manifest()

  def write_manifest(self):
    (self.bundle/'manifest.json').write_text(json.dumps(self.manifest))

  def ready(self):
    return runtime.runtime_directory(self.bundle,self.base,check_platform=False)

  def install(self):
    return runtime.prepare_runtime(self.bundle,self.base,check_platform=False)

  def test_install_once_then_skip_extraction(self):
    self.assertEqual(self.install(),'installed')
    self.assertIsNotNone(self.ready())
    with patch.object(runtime,'_extract_wheel',side_effect=AssertionError('reinstalled')):
      self.assertEqual(self.install(),'ready')

  def test_changed_manifest_and_broken_install_are_repaired(self):
    self.install()
    original = self.ready()
    self.manifest['version'] = '2'
    self.write_manifest()
    self.assertIsNone(self.ready())
    self.assertEqual(self.install(),'installed')
    updated = self.ready()
    self.assertNotEqual(original,updated)
    (updated/'onnxruntime/capi/pybind.so').write_bytes(b'broken')
    self.assertIsNone(self.ready())
    self.assertEqual(self.install(),'installed')

  def test_failed_update_preserves_previous_runtime(self):
    self.install()
    old_pointer=(self.base/'active.json').read_bytes()
    self.manifest['version']='2'
    self.write_manifest()
    with patch.object(runtime,'_extract_wheel',side_effect=OSError('interrupted')):
      with self.assertRaises(OSError):
        self.install()
    self.assertEqual((self.base/'active.json').read_bytes(),old_pointer)
    self.assertEqual(list(self.base.glob('.staging-*')),[])

  def test_checksum_mismatch_never_activates(self):
    self.manifest['wheels'][0]['sha256']='0'*64
    self.write_manifest()
    with self.assertRaises(ValueError):
      self.install()
    self.assertFalse((self.base/'active.json').exists())

  def test_archive_and_pointer_cannot_escape_runtime_directory(self):
    self.make_bundle(bad_path=True)
    with self.assertRaises(ValueError):
      self.install()
    self.assertFalse((self.base/'escaped.py').exists())
    self.base.mkdir(exist_ok=True)
    (self.base/'active.json').write_text(json.dumps(dict(directory='../outside')))
    self.assertIsNone(self.ready())

  def test_unsupported_platform_does_not_install(self):
    with patch.object(runtime,'supported',return_value=False):
      self.assertEqual(runtime.prepare_runtime(self.bundle,self.base),'unsupported')
    self.assertFalse(self.base.exists())

  def test_import_failure_marker_requires_repair_next_boot(self):
    self.install()
    directory=self.ready()
    with patch.dict('os.environ',{'LEAD_BRAKE_RUNTIME_DIR':str(self.base)}):
      runtime.mark_broken(directory)
    self.assertIsNone(self.ready())
    self.assertEqual(self.install(),'installed')
    self.assertIsNotNone(self.ready())


if __name__=='__main__':
  unittest.main()
