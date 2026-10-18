"""A lower weighted score cannot hide slower acquisition or failed recovery."""
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import score

def group(name,value=1, rise=8):
    return [dict(variant=name,case=i,score=value,gates=[],shaft_Wh_per_ground_km=4,
        tasks=dict(acquisition=value,descent=value,updraft=value,crosswind=value),
        steps={'climb':{'rise_10_90_s':rise}}) for i in [1,3,13]]

class SelectionTests(unittest.TestCase):
    def select(self,rows):
        with tempfile.TemporaryDirectory() as directory:
            with patch.object(score,'HERE',Path(directory)),patch.object(score,'analyze_phase',return_value=rows):
                return score.select(Path(directory))

    def test_slow_cannot_win(self):
        result=self.select(group('fixed')+group('slow',.3,9)+group('good',.8))
        self.assertEqual(result['selected'],'good')
        self.assertIn('acquisition speed regression >10%',result['candidates']['slow']['reasons'])

    def test_failed_cannot_win(self):
        bad=group('failed',.1);bad[0].update(score=None,gates=['unrecovered gust'])
        self.assertEqual(self.select(group('fixed')+bad)['selected'],'fixed')

    def test_recovery_regression_cannot_hide(self):
        bad=group('bad',.3)
        for r in bad:r['tasks']['updraft']=1.3
        self.assertEqual(self.select(group('fixed')+bad)['selected'],'fixed')

if __name__=='__main__':unittest.main()
