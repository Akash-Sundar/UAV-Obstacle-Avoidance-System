import contextlib
import importlib
import inspect
import json
from pathlib import Path
import sys
import time
import unittest

import numpy as np
from scipy.spatial.transform import Rotation
from numpy.linalg import norm

from flightsim.numpy_encoding import NumpyJSONEncoder

def complementary_filter_update_test(d, complementary_filter_update):
    # Run a test
    R0 = Rotation.from_quat(d['initial_rotation'])
    w = np.array(d['angular_velocity'])
    a = np.array(d['linear_acceleration'])
    dt = d['dt']
    output_rotation = Rotation.from_quat(d['output_rotation'])

    rout = complementary_filter_update(R0, w, a, dt)

    temp = rout.inv() * output_rotation
    error = temp.magnitude()
    tol = 1e-4

    res = {'computed_rotation': rout.as_quat().tolist(),
           'true_rotation': output_rotation.as_quat().tolist(),
           'error': error, 'tol': tol, 'passed': bool(error < tol)}
    return res


class TestBase(unittest.TestCase):

    complementary_filter_update_cls = None

    longMessage = False
    outpath = Path(__file__).resolve().parent.parent / 'data_out'
    outpath.mkdir(parents=True, exist_ok=True)

    test_names = []

    def helper_test(self, test_name, test_file, std_target):
        """
        Test student code against provided test and save results to file.
        """
        with contextlib.redirect_stdout(std_target):  # gobbles stdout.
            with open(test_file) as tf:
                test_dict = json.load(tf)

                test_fcn = globals()[test_dict['function_to_test'] + '_test']
                target_fcn = getattr(TestBase, test_dict['function_to_test'] + '_cls')
                results = test_fcn(test_dict, target_fcn)
                results['test_type'] = test_dict['function_to_test']

                result_file = self.outpath / ('result_' + test_name + '.json')
                with open(result_file, 'w') as rf:
                    rf.write(json.dumps(results, cls=NumpyJSONEncoder, indent=4))

    @classmethod
    def set_target(cls, module_name):
        """
        Set the target module to test and load required classes or functions.
        """
        cls.complementary_filter_update_cls = importlib.import_module(
            module_name + '.complementary_filter').complementary_filter_update

    @classmethod
    def load_tests(cls, files, *, enable_timeouts=False, redirect_stdout=True):
        """
        Add one test for each input file. For each input file named
        "test_XXX.json" creates a new test member function that will generate
        output files "result_XXX.json" and "result_XXX.pdf".
        """
        std_target = None if redirect_stdout else sys.stdout
        for file in files:
            if file.stem.startswith('test_') and file.suffix == '.json':
                test_name = file.stem[5:]
                cls.test_names.append(test_name)

                # create class member function test_* to be executed by unittest
                def fn(self, test_name=test_name, test_file=file):
                    self.helper_test(test_name, test_file, std_target)
                setattr(cls, 'test_' + test_name, fn)

                # delete existing results file
                with contextlib.suppress(FileNotFoundError):
                    (cls.outpath / ('result_' + test_name + '.json')).unlink()
                with contextlib.suppress(FileNotFoundError):
                    (cls.outpath / ('result_' + test_name + '.pdf')).unlink()

    @classmethod
    def collect_results(cls):
        results = []
        for name in cls.test_names:
            p = cls.outpath / ('result_' + name + '.json')
            if p.exists():
                with open(p) as f:
                    data = json.load(f)
                    data['test_name'] = name
                    results.append(data)
            else:
                results.append({'test_name': name})
        return results

    @classmethod
    def print_results(cls):
        results = cls.collect_results()
        for r in results:
            if not 'passed' in r.keys():
                print('{} {}'.format('FAIL', r['test_name']))
                print('    no results produced')
                continue
            if r['passed']:
                print('{} {}'.format('PASS', r['test_name']))
                continue
            print('{} {}'.format('FAIL', r['test_name']))
            print('  rotation error exceeds tolerance: {:.4f} > {:.0e}'.format(r['error'], r['tol']))


if __name__ == '__main__':
    """
    Run a test for each "test_*.json" file in this directory. You can add new
    tests by copying and editing these files.
    """
    import argparse

    # All arguments are optional, and are not needed to test the student solution.
    default_target = 'proj2_1.code'
    parser = argparse.ArgumentParser(description='Evaluate one assignment solution.')
    parser.add_argument('--target', default=default_target, type=str,
                        help=f"Run on the code module of this name. Default is {default_target}")
    parser.add_argument('--stdout', action='store_true',
                        help="Allow printing to stdout from inside unittest.")
    p = parser.parse_args()

    if p.stdout:
        print('\n*** WARNING: ENABLED PRINTING TO STDOUT FROM INSIDE UNITTEST ***\n')

    # Set target code module to test.
    if p.target != default_target:
        print(f'\n*** WARNING: RUNNING IN DEBUG MODE USING MODULE {p.target} ***\n')
    TestBase.set_target(module_name=p.target)

    # Collect tests distributed to students.
    path = Path(inspect.getsourcefile(TestBase)).parent.resolve()
    test_files_local = list(Path(path).glob('test_*.json'))
    # Concatenate full list of tests.
    all_test_files = test_files_local
    # load test in order they are processed by unittest so that test results
    # are printed in the same order that they are processed
    all_test_files.sort()
    TestBase.load_tests(all_test_files, redirect_stdout=not p.stdout)

    # Run tests, results saved in data_out.
    suite = unittest.TestLoader().loadTestsFromTestCase(TestBase)
    unittest.TextTestRunner(verbosity=2).run(suite)

    # Collect results for display.
    TestBase.print_results()
