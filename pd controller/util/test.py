import contextlib
import importlib
import inspect
import json
import os
from pathlib import Path
import sys
import unittest

import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.distance import cdist

from flightsim.numpy_encoding import to_ndarray
from flightsim.simulate import Quadrotor, simulate, ExitStatus
from flightsim.world import World
from flightsim.axes3ds import Axes3Ds
from flightsim.crazyflie_params import quad_params

def test_mission(waypoint_traj_cls, se3_control_cls, points, initial_state):
    """
    Test the provided trajectory class and control class against a set of
    waypoints and a specific initial state. Return the simulation results and
    the performance metrics.
    """

    # Student code to test.
    my_traj = waypoint_traj_cls(points)
    my_se3_control = se3_control_cls(quad_params)

    # Simulation options.
    quadrotor = Quadrotor(quad_params)
    t_final = 60

    # Run simulation.
    (time, state, control, flat, exit) = simulate(
        initial_state,
        quadrotor,
        my_se3_control,
        my_traj,
        t_final)
    results = {'time':time, 'state':state, 'control':control, 'flat':flat, 'exit':exit}

    # Evaluate results.
    metrics = {}
    # Must come to rest at goal.
    metrics['stopped_at_goal'] = (results['exit'] == ExitStatus.COMPLETE)
    metrics['time'] = time[-1]
    # Must pass within minimum distance of each waypoint.
    dist = cdist(points, results['state']['x'])
    min_dist = np.min(dist, axis=1)
    metrics['reached_waypoints'] = bool(np.max(min_dist) < 0.5)
    # Details about why the simulation ended (success, failure, timeout).
    metrics['sim_exit'] = results['exit'].value

    return (results, metrics)

def plot_mission(points, results, title):
    """
    Return a figure showing simulation results along with a set of waypoints,
    formatting for printing as a letter size .pdf.
    """

    import matplotlib.pyplot as plt
    from matplotlib.axes import Axes
    from matplotlib.projections import register_projection

    from flightsim.axes3ds import Axes3Ds

    register_projection(Axes3Ds)

    def _decimate_to_freq(time, freq):
        """

        Given sorted lists of source times and a target sampling frequency in Hz,
        return indices of source times to approximate frequency.

        """
        if time[-1] != 0:
            sample_time = np.arange(0, time[-1], 1/freq)
        else:
            sample_time = np.zeros((1,))
        index = np.arange(time.size)
        sample_index = np.round(np.interp(sample_time, time, index)).astype(int)
        sample_index = np.unique(sample_index)
        return sample_index

    idx = _decimate_to_freq(results['time'], 100)

    fig = plt.figure(num=str(title), figsize=(8.5,11.0), clear=True)
    fig.suptitle(str(title))

    # Build world that fits path results.
    margin = 0.1
    pts = np.concatenate((results['state']['x'], results['flat']['x']), axis=0)
    a = np.min(pts, axis=0)-margin
    b = np.max(pts, axis=0)+margin
    b = a + np.max(b - a)
    world = World.empty((a[0], b[0], a[1], b[1], a[2], b[2]))

    # 3D Position
    x = results['state']['x'][idx,:]
    x_des = results['flat']['x'][idx,:]
    ax = fig.add_subplot(2, 2, 1, projection='3d')
    world.draw(ax)
    ax.plot3D(x[:,0], x[:,1], x[:,2], 'b.')
    ax.plot3D(x_des[:,0], x_des[:,1], x_des[:,2], 'k')

    # Position vs. Time
    x = results['state']['x'][idx,:]
    x_des = results['flat']['x'][idx,:]
    time = results['time'][idx]
    ax = fig.add_subplot(2, 2, 2)
    ax.plot(time, x_des[:,0], 'r', time, x_des[:,1], 'g', time, x_des[:,2], 'b')
    ax.plot(time, x[:,0], 'r.',    time, x[:,1], 'g.',    time, x[:,2], 'b.')
    ax.legend(('x', 'y', 'z'))
    ax.set_xlabel('time, s')
    ax.set_ylabel('position, m')
    ax.grid('major')
    ax.set_title('Position')

    # Motor speeds.
    ax = fig.add_subplot(2, 2, 3)
    s = results['control']['cmd_motor_speeds'][idx,:]
    ax.plot(time, s[:,0], 'r.', time, s[:,1], 'g.', time, s[:,2], 'b.', time, s[:,3], 'k.')
    ax.legend(('1', '2', '3', '4'))
    ax.set_xlabel('time, s')
    ax.set_ylabel('motor speeds, rad/s')
    ax.grid('major')
    ax.set_title('Commands')

    # Orientation.
    ax = fig.add_subplot(2, 2, 4)
    q_des = results['control']['cmd_q'][idx,:]
    q = results['state']['q'][idx,:]
    ax.plot(time, q_des[:,0], 'r', time, q_des[:,1], 'g', time, q_des[:,2], 'b', time, q_des[:,3], 'k')
    ax.plot(time, q[:,0], 'r.',    time, q[:,1], 'g.',    time, q[:,2], 'b.',    time, q[:,3],     'k.')
    ax.legend(('i', 'j', 'k', 'w'))
    ax.set_xlabel('time, s')
    ax.set_ylabel('quaternion')
    ax.grid('major')
    ax.set_title('Orientation')

    return fig

class TestBase(unittest.TestCase):

    waypoint_traj_cls = None
    se3_control_cls = None

    longMessage = False
    outpath = Path(inspect.getsourcefile(test_mission)).resolve().parent.parent / 'data_out'
    outpath.mkdir(parents=True, exist_ok=True)

    test_names = []

    def helper_test_traj(self, test_name, points, initial_state, std_target):
        """
        Test student's SE3Control and WaypointTraj against provided waypoints.
        Run simulation, save metrics to file, save result plots to file.
        """
        with contextlib.redirect_stdout(std_target):  # Context gobbles stdout.
            (results, metrics) = test_mission(self.waypoint_traj_cls,
                                              self.se3_control_cls,
                                              points,
                                              initial_state)
            with open(self.outpath / ('result_' + test_name + '.json'), 'w') as f:
                json.dump(metrics, f, indent=4, separators=(',', ': '))
            fig = plot_mission(points, results, test_name)
            fig.savefig(self.outpath / ('result_' + test_name + '.pdf'))

    @classmethod
    def set_target(cls, module_name):
        """
        Set the target module to test, and load required classes or functions.
        """
        cls.waypoint_traj_cls = importlib.import_module(module_name + '.waypoint_traj').WaypointTraj
        cls.se3_control_cls = importlib.import_module(module_name + '.se3_control').SE3Control

    @classmethod
    def load_tests(cls, files, redirect_stdout=True):
        """
        Add one test for each input file. For each input file named
        "test_XXX.json" creates a new test member function that will generate
        output files "result_XXX.json" and "result_XXX.pdf".
        """
        std_target = None if redirect_stdout else sys.stdout
        for file in files:
            if file.stem.startswith('test_'):
                test_name = file.stem[5:]
                cls.test_names.append(test_name)
                with open(file) as f:
                    data = to_ndarray(json.load(f))
                # Dynamically add member function for this test.
                def fn(self, test_name=test_name,
                             points=data['points'],
                             initial_state=data['initial_state']):
                    self.helper_test_traj(test_name, points, initial_state, std_target)
                setattr(cls, 'test_'+test_name, fn)
                # Remove any pre-existing output files for this test.
                # TODO: The 'missing_ok' argument becomes available in Python
                # 3.8, at which time contextlib is no longer needed.
                with contextlib.suppress(FileNotFoundError):
                    (cls.outpath / ('result_' + test_name + '.json')).unlink()
                with contextlib.suppress(FileNotFoundError):
                    (cls.outpath / ('result_' + test_name + '.pdf')).unlink()

    def test_unit_traj_start_loc(self):
        with contextlib.redirect_stdout(None): # Context gobbles stdout.
            # Check that the return value of the trajectory at time t=0 is correct.
            points = np.array([[1.0, 2.0, 3.0], [10, 20, 30]])
            traj = self.waypoint_traj_cls(points)
            flat_outputs = traj.update(0)
            self.assertIsInstance(flat_outputs, dict)
            self.assertIsInstance(flat_outputs['x'], np.ndarray)
            self.assertEqual(flat_outputs['x'].shape, (3,))
            self.assertTrue(np.allclose(flat_outputs['x'], points[0,:], atol=1e-2),
                msg='Trajectory at time t=0 does not return the start waypoint location.')

    def test_unit_traj_end_loc(self):
        with contextlib.redirect_stdout(None): # Context gobbles stdout.
            # Check that the return value of the trajectory at time t=np.inf is correct.
            points = np.array([[1.0, 2.0, 3.0], [10, 20, 30]])
            traj = self.waypoint_traj_cls(points)
            flat_outputs = traj.update(np.inf)
            self.assertIsInstance(flat_outputs, dict)
            self.assertIsInstance(flat_outputs['x'], np.ndarray)
            self.assertEqual(flat_outputs['x'].shape, (3,))
            self.assertTrue(np.allclose(flat_outputs['x'], points[-1,:], atol=1e-2),
                msg='Trajectory at time t=inf does not return the end waypoint location.')

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
                results.append({'test_name':name})
        return results

    @classmethod
    def print_results(cls):
        results = cls.collect_results()
        for r in results:
            if 'time' in r:
                wp = r['reached_waypoints']
                g  = r['stopped_at_goal']
                print("{pf} {name}\n"
                      "    reached points:  {wp}\n"
                      "    stopped at goal: {g}\n"
                      "    time:            {t:.3f}\n"
                      "    exit message:    {e}".format(
                        name=r['test_name'],
                        pf='pass' if wp and g else 'FAIL',
                        wp='ok' if wp else 'FAIL',
                        g ='ok' if g  else 'FAIL',
                        t=r['time'],
                        e=r['sim_exit']))
            else:
                print("FAIL {name}\n"
                      "    Test failed with no results. Review error log.".format(
                        name=r['test_name']))

if __name__ == '__main__':
    """
    Run a test for each "test_*.json" file in this directory. You can add new
    tests by copying and editing these files.
    """

    import argparse

    # All arguments are optional, and are not needed to test the student solution.
    default_target = 'proj1_1.code'
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
        print(f'\n*** WARNING: RUNNING IN DEBUG MODE USING MODULE {p.target}) ***\n')
    TestBase.set_target(module_name=p.target)

    # Collect tests distributed to students.
    path = Path(inspect.getsourcefile(TestBase)).parent.resolve()
    test_files_local = list(Path(path).glob('test_*.json'))
    # Concatenate full list of tests.
    all_test_files = test_files_local
    TestBase.load_tests(all_test_files, redirect_stdout=not p.stdout)

    # Run tests, results saved in data_out.
    suite = unittest.TestLoader().loadTestsFromTestCase(TestBase)
    unittest.TextTestRunner(verbosity=2).run(suite)

    # Collect results for display.
    TestBase.print_results()