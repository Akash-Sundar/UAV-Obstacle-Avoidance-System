import contextlib
import importlib
import inspect
import json
from pathlib import Path
import re
import sys
import time
import unittest

import numpy as np
from scipy.spatial.transform import Rotation
from numpy.linalg import norm


def error_covariance_update_test(d, vio):
    in_p = np.array(d['in_p'])
    in_v = np.array(d['in_v'])
    in_q = Rotation.from_quat(d['in_q'])
    in_a_b = np.array(d['in_a_b'])
    in_w_b = np.array(d['in_w_b'])
    in_g = np.array(d['in_g'])

    in_error_state_covariance = np.array(d['in_error_state_covariance'])

    w_m = np.array(d['w_m'])
    a_m = np.array(d['a_m'])
    dt = d['dt']

    accelerometer_noise_density = d['accelerometer_noise_density']
    gyroscope_noise_density = d['gyroscope_noise_density']
    accelerometer_random_walk = d['accelerometer_random_walk']
    gyroscope_random_walk = d['gyroscope_random_walk']

    out_error_state_covariance = np.array(d['out_error_state_covariance'])

    nominal_state = in_p, in_v, in_q, in_a_b, in_w_b, in_g

    # Run a test
    error_state_covariance = vio.error_covariance_update(nominal_state, in_error_state_covariance, w_m, a_m, dt,
                                                         accelerometer_noise_density, gyroscope_noise_density,
                                                         accelerometer_random_walk, gyroscope_random_walk)

    delta = error_state_covariance - out_error_state_covariance

    cov_err = norm(delta.astype(float).ravel())
    cov_err_tol = 1e-5
    res = {'cov_err': cov_err, 'cov_err_tol': cov_err_tol,
           'cov_passed': bool(cov_err < cov_err_tol), # match format used for other tests
           'passed': bool(cov_err < cov_err_tol)}
    return res


def nominal_state_update_test(d, vio):
    in_p = np.array(d['in_p'])
    in_v = np.array(d['in_v'])
    in_q = Rotation.from_quat(d['in_q'])
    in_a_b = np.array(d['in_a_b'])
    in_w_b = np.array(d['in_w_b'])
    in_g = np.array(d['in_g'])

    w_m = np.array(d['w_m'])
    a_m = np.array(d['a_m'])
    dt = d['dt']

    out_p = np.array(d['out_p'])
    out_v = np.array(d['out_v'])
    out_q = Rotation.from_quat(d['out_q'])
    out_a_b = np.array(d['out_a_b'])
    out_w_b = np.array(d['out_w_b'])
    out_g = np.array(d['out_g'])

    nominal_state = in_p, in_v, in_q, in_a_b, in_w_b, in_g

    # Run a test
    p, v, q, a_b, w_b, g = vio.nominal_state_update(nominal_state, w_m, a_m, dt)

    p_err = norm(out_p - p)
    v_err = norm(out_v - v)
    delta = out_q.inv() * q
    q_err = delta.magnitude()
    a_b_err = norm(out_a_b - a_b)
    w_b_err = norm(out_w_b - w_b)
    g_err = norm(out_g - g)

    test_vars = [ 'p',  'v',  'q', 'a_b', 'w_b',  'g']
    test_tols = [1e-5, 1e-5, 1e-4,  1e-5,  1e-5, 1e-5]
    res = {}
    for v, tol in zip(test_vars, test_tols):
        res[v+'_err'] = locals()[v+'_err']
        res[v+'_err_tol'] = tol
        res[v+'_passed'] = bool(res[v+'_err'] < tol)
    res['passed'] = all([res[v+'_passed'] for v in test_vars])

    return res


def measurement_update_step_test(d, vio):
    in_p = np.array(d['in_p'])
    in_v = np.array(d['in_v'])
    in_q = Rotation.from_quat(d['in_q'])
    in_a_b = np.array(d['in_a_b'])
    in_w_b = np.array(d['in_w_b'])
    in_g = np.array(d['in_g'])

    in_error_state_covariance = np.array(d['in_error_state_covariance'])

    uv = np.array(d['uv'])
    Pw = np.array(d['Pw'])
    error_threshold = d['error_threshold']
    image_measurement_covariance = np.array(d['image_measurement_covariance'])

    out_p = np.array(d['out_p'])
    out_v = np.array(d['out_v'])
    out_q = Rotation.from_quat(d['out_q'])
    out_a_b = np.array(d['out_a_b'])
    out_w_b = np.array(d['out_w_b'])
    out_g = np.array(d['out_g'])

    out_error_state_covariance = np.array(d['out_error_state_covariance'])

    out_inno = np.array(d['out_inno'])

    nominal_state = in_p, in_v, in_q, in_a_b, in_w_b, in_g

    # Run a test
    nominal_state, error_state_covariance, inno = vio.measurement_update_step(nominal_state,
                                                                              in_error_state_covariance,
                                                                              uv, Pw, error_threshold,
                                                                              image_measurement_covariance)

    p, v, q, a_b, w_b, g = nominal_state

    p_err = norm(out_p - p)
    v_err = norm(out_v - v)
    delta = out_q.inv() * q
    q_err = delta.magnitude()
    a_b_err = norm(out_a_b - a_b)
    w_b_err = norm(out_w_b - w_b)
    g_err = norm(out_g - g)
    inno_err = norm(out_inno - inno)
    delta = error_state_covariance - out_error_state_covariance
    cov_err = norm(delta.ravel())

    test_vars = [ 'p',  'v',  'q', 'a_b', 'w_b',  'g', 'inno', 'cov']
    test_tols = [1e-5, 1e-5, 1e-4,  1e-5,  1e-5, 1e-5,   1e-5,  1e-5]
    res = {}
    for v, tol in zip(test_vars, test_tols):
        res[v+'_err'] = locals()[v+'_err']
        res[v+'_err_tol'] = tol
        res[v+'_passed'] = bool(res[v+'_err'] < tol)
    res['passed'] = all([res[v+'_passed'] for v in test_vars])

    return res


class TestBase(unittest.TestCase):

    vio_cls = None

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
                td = json.load(tf)

                test_fcn = globals()[td['function_to_test'] + '_test']
                results = test_fcn(td, self.vio_cls)
                results['test_type'] = td['function_to_test']

                result_file = self.outpath / ('result_' + test_name + '.json')
                with open(result_file, 'w') as rf:
                    rf.write(json.dumps(results, indent=4))

    @classmethod
    def set_target(cls, module_name):
        """
        Set the target module to test and load required classes or functions.
        """
        cls.vio_cls = importlib.import_module(module_name + '.vio')

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
        # TODO add printout
        for r in results:
            if not 'passed' in r.keys():
                print('{} {}'.format('FAIL', r['test_name']))
                print('  no results produced')
                continue
            if r['passed']:
                print('{} {}'.format('PASS', r['test_name']))
                continue
            print('{} {}'.format('FAIL', r['test_name']))
            re_pass = re.compile(r'.*_passed$')
            re_var = re.compile(r'^.*(?=_passed$)')
            pass_keys = [k for k in r.keys() if re_pass.match(k) is not None]
            for v in [re_var.match(k).group(0) for k in pass_keys if not r[k]]:
                print('  {:.2e} ({}) > {:.0e} ({})'.format(r[v+'_err'], v+'_err',
                                                           r[v+'_err_tol'], v+'_err_tol'))


if __name__ == '__main__':
    """
    Run a test for each "test_*.json" file in this directory. You can add new
    tests by copying and editing these files.
    """
    import argparse

    # All arguments are optional, and are not needed to test the student solution.
    default_target = 'proj2_3.code'
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
