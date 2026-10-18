"""Selection must not trade away recovery or admit failed runs for a lower mean."""
import copy
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import score


def group(name, value=1.):
    return [dict(variant=name,case=i,score=value,gates=[],shaft_Wh_per_ground_km=4.,
                 tasks=dict(acquisition=value,descent=value,updraft=value,crosswind=value)) for i in [1,3,13]]


class SelectionTests(unittest.TestCase):
    def run_selection(self, rows):
        with tempfile.TemporaryDirectory() as directory:
            with patch.object(score,'HERE',Path(directory)), patch.object(score,'analyze_phase',return_value=rows):
                return score.select(Path(directory))

    def test_regression_rejects_lower_aggregate(self):
        bad=group('bad',.5)
        for r in bad:r['tasks']['updraft']=1.3
        result=self.run_selection(group('fixed')+bad+group('good',.8))
        self.assertEqual(result['selected'],'good')
        self.assertTrue(result['candidates']['bad']['reasons'])

    def test_failed_case_cannot_win(self):
        bad=group('bad',.1);bad[0].update(score=None,gates=['incomplete flight'])
        self.assertEqual(self.run_selection(group('fixed')+bad)['selected'],'fixed')

    def test_small_improvement_does_not_promote(self):
        self.assertEqual(self.run_selection(group('fixed')+group('noise',.98))['selected'],'fixed')


if __name__=='__main__':unittest.main()
