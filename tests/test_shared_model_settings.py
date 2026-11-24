"""Nonsecret model configuration across device sessions; no model/hardware calls."""
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from franka_player.core import Console,MODEL_FIELDS
from robol2l_deploy.cap.model import ConfiguredModel
from robol2l_deploy.localization import chinese_error


class SharedModelSettingsTests(unittest.TestCase):
    def test_migrate_launcher_settings_and_share_edits_without_device_state(self):
        with tempfile.TemporaryDirectory() as folder,patch.dict(os.environ,{},clear=True):
            root=Path(folder);launcher=Console(root/'launcher-console')
            launcher.configure(dict(base_url='https://example.com/v1',model='fixture-model',reasoning='medium',station='launcher-only'))
            path=root/'model-settings.json'
            first=Console(root/'sessions/first',model_settings_path=path)
            second=Console(root/'sessions/second',model_settings_path=path)
            self.assertEqual(first.snapshot()['config']['base_url'],'https://example.com/v1')
            self.assertEqual(second.snapshot()['config']['model'],'fixture-model')
            self.assertEqual(first.snapshot()['config']['station'],'')
            first.configure({'model':'changed-model'})
            self.assertEqual(second.snapshot()['config']['model'],'changed-model')
            reloaded=Console(root/'sessions/first',model_settings_path=path)
            self.assertEqual(reloaded.snapshot()['config']['model'],'changed-model')
            self.assertEqual(set(json.loads(path.read_text())),set(MODEL_FIELDS))

    def test_environment_secret_is_never_written_to_shared_settings(self):
        with tempfile.TemporaryDirectory() as folder,patch.dict(os.environ,{'ROBOL2L_MODEL_BASE_URL':'https://example.com/v1','ROBOL2L_MODEL':'fixture','ROBOT_MODEL_API_KEY':'test-only-secret'},clear=True):
            path=Path(folder)/'model-settings.json';console=Console(Path(folder)/'session',model_settings_path=path)
            self.assertEqual(console.snapshot()['config']['base_url'],'https://example.com/v1')
            for file in Path(folder).rglob('*.json'):self.assertNotIn('test-only-secret',file.read_text())

    def test_missing_endpoint_name_and_malformed_url_have_specific_chinese_feedback(self):
        base=dict(base_url='https://example.com/v1',model='fixture',reasoning='medium',api_key_env='ROBOT_MODEL_API_KEY')
        for change,error in [({'base_url':''},'model_endpoint_missing'),({'model':''},'model_name_missing'),
                             ({'base_url':'missing-scheme'},'invalid_model_endpoint'),({'base_url':'https://example.com:bad/v1'},'invalid_model_endpoint')]:
            with self.subTest(change=change):
                with self.assertRaisesRegex(ValueError,error):ConfiguredModel(dict(base,**change))
                self.assertNotIn('操作未完成，请查看错误详情',chinese_error(error))
        self.assertEqual(ConfiguredModel(dict(base,base_url=' https://example.com/v1 ')).config['base_url'],base['base_url'])


if __name__=='__main__':unittest.main()
