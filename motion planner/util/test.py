import contextlib
import importlib
import inspect
import json
import os
from pathlib import Path
import sys
import time
import timeout_decorator
import unittest

import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import numpy as np
from scipy.spatial.distance import cdist

from flightsim.world import World

def set_path_metrics(metrics, path_name, path, time, world, start, goal, resolution, margin, expected_path_length):
    """
    Set path metrics in one easy function!
    :return:
        metrics filled for the given path.
    """

    metrics[path_name] = {}
    metrics[path_name]['time'] = time

    eps = 1e-3

    if path is not None:
        metrics[path_name]['path_length'] = float(round(np.sum(np.linalg.norm(np.diff(path, axis=0),axis=1)),3))
        metrics[path_name]['reached_start'] = bool(np.linalg.norm(path[0] - start) <= 1e-3)
        metrics[path_name]['reached_goal'] = bool(np.linalg.norm(path[-1] - goal) <= 1e-3)
        metrics[path_name]['no_collision'] = world.path_collisions(path, margin).size == 0
    else:
        metrics[path_name]['path_length'] = np.inf
        metrics[path_name]['reached_start'] = False
        metrics[path_name]['reached_goal'] = False
        metrics[path_name]['no_collision'] = False

    if expected_path_length is not None:
        if expected_path_length == np.inf and metrics[path_name]['path_length'] == np.inf:
            metrics[path_name]['is_optimal'] = True
        else:
            # The length criterion is based on the shortest path along voxel
            # centers from the center of the start voxel to the center of the
            # end voxel plus connections to the true start point at the
            # beginning and the true goal point at the end. The non-unique
            # choice of starting voxel for a start point on a voxel corner adds
            # at most a voxel diagonal length to the path, and similarly the
            # non-unique goal connection adds at most another voxel diagonal.
            metrics[path_name]['is_optimal'] = bool(metrics[path_name]['path_length'] <= expected_path_length + 2*np.linalg.norm(resolution))
    else:
        metrics[path_name]['is_optimal'] = None  # Solution length not available for student-written tests.
    return metrics


def test_mission(graph_search_fn, occupancy_map, world, start, goal, resolution, margin, expected_path_length, algorithms):
    """
    Test the provided graph_search function against a world, start, and goal.
    Return the simulation results and the performance metrics.
    """
    dijkstra = bool([s for s in algorithms if "dijkstra" in s])
    astar = bool([s for s in algorithms if "astar" in s])

    # Run student code.
    oc = occupancy_map(world, resolution, margin)
    results = {}
    metrics = {}

    if dijkstra:
        start_time = time.time()
        results['dijkstra_path'], _ = graph_search_fn(world, resolution, margin, start, goal, False)
        dijkstra_time = round(time.time() - start_time, 3)
        set_path_metrics(metrics, 'dijkstra', results['dijkstra_path'], dijkstra_time, world, start, goal, resolution, margin, expected_path_length)

    if astar:
        start_time = time.time()
        results['astar_path'], _ = graph_search_fn(world, resolution, margin, start, goal, True)
        astar_time = round(time.time() - start_time, 3)
        set_path_metrics(metrics, 'astar', results['astar_path'], astar_time, world, start, goal, resolution, margin, expected_path_length)

    w = world.world['bounds']['extents']
    metrics['map_nodes'] = int(np.prod(np.round((w[1::2]-w[0::2])/resolution)))

    return results, metrics


def plot_mission(world, start, goal, results, test_name):
    """
    Return a figure showing path through trees along with start and end.
    """

    import matplotlib.pyplot as plt
    from matplotlib.axes import Axes
    from matplotlib.projections import register_projection

    from flightsim.axes3ds import Axes3Ds

    def plot_path(path, path_type):
        fig = plt.figure()
        ax = Axes3Ds(fig)
        world.draw(ax)
        if path is not None:
            world.draw_line(ax, path, color='blue')
            world.draw_points(ax, path, color='blue')
        ax.plot([start[0]], [start[1]], [start[2]], 'go', markersize=10, markeredgewidth=3, markerfacecolor='none')
        ax.plot( [goal[0]],  [goal[1]],  [goal[2]], 'ro', markersize=10, markeredgewidth=3, markerfacecolor='none')
        ax.set_title("%s Path through %s" % (path_type, test_name))
        return fig

    fig_list = []
    if 'dijkstra_path' in results:
        dijkstra_path = results['dijkstra_path']
        dijkstra_fig = plot_path(dijkstra_path, "Dijkstra's")
        fig_list.append(dijkstra_fig)

    if 'astar_path' in results:
        astar_path = results['astar_path']
        astar_fig = plot_path(astar_path, "A*")
        fig_list.append(astar_fig)

    return fig_list

class TestBase(unittest.TestCase):
    graph_search_fn = None
    occupancy_map_cls = None

    longMessage = False
    outpath = Path(inspect.getsourcefile(test_mission)).resolve().parent.parent / 'data_out'
    outpath.mkdir(parents=True, exist_ok=True)

    test_names = []

    def helper_test(self, test_name, world, start, goal, resolution, margin, expected_path_length, std_target, algorithms):
        """
        Test student's graph_search against given world, start, and goal.
        Run solution, save metrics to file, save result plots to file.
        """
        with contextlib.redirect_stdout(std_target):  # Context gobbles stdout.
            result_file = self.outpath / ('result_' + test_name + '.json')
            try: # gracefully handle timeout exceptions
                (results, metrics) = test_mission(self.graph_search_fn, self.occupancy_map_cls, world, start, goal,
                                                  resolution, margin, expected_path_length, algorithms)
                with open(result_file, 'w') as f:
                    json.dump(metrics, f, indent=4, separators=(',', ': '))
                    figs = plot_mission(world, start, goal, results, test_name)
                    # Save all figures to file
                    with PdfPages(self.outpath / ('result_' + test_name + '.pdf')) as pdf:
                        for fig in figs:
                            pdf.savefig(fig)
            except timeout_decorator.timeout_decorator.TimeoutError as err:
                with open(result_file, 'w') as f:
                    output = {'test_name': test_name, 'error': err.value}
                    json.dump(output, f, indent=4, separators=(',', ': '))

    @classmethod
    def set_target(cls, module_name):
        """
        Set the target module to test, and load required classes or functions.
        """
        cls.graph_search_fn = staticmethod(importlib.import_module(module_name + '.graph_search').graph_search)
        cls.occupancy_map_cls = importlib.import_module(module_name + '.occupancy_map').OccupancyMap

    @classmethod
    def load_tests(cls, files, enable_timeouts=False, redirect_stdout=True):
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
                world=World.from_file(file)

                timeout = None
                algorithms = None
                with open(file) as f:
                    test_dict = json.load(f)
                    if enable_timeouts:
                        if 'timeout' in test_dict.keys():
                            timeout = test_dict['timeout']
                    algorithms = test_dict['algorithms']
                # Dynamically add member function for this test.
                @timeout_decorator.timeout(timeout, exception_message="Test reached time limit of {} seconds".format(timeout))
                def fn(self, test_name=test_name,
                       world=world,
                       start=world.world['start'],
                       goal=world.world['goal'],
                       resolution=world.world['resolution'],
                       margin=world.world['margin'],
                       expected_path_length=world.world.get('expected_path_length', None), algorithms=algorithms):
                    self.helper_test(test_name, world, start, goal, resolution, margin, expected_path_length, std_target, algorithms)

                setattr(cls, 'test_' + test_name, fn)
                # Remove any pre-existing output files for this test.
                # TODO: The 'missing_ok' argument becomes available in Python
                # 3.8, at which time contextlib is no longer needed.
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
            print()  # prettiness
            if len(r.keys()) > 2:
                if r['test_name'] != 'impossible':
                    dijkstra_optimal = ('dijkstra' in r) and (r['dijkstra']['is_optimal'] or r['dijkstra']['is_optimal'] == None)
                    astar_optimal = ('astar' in r) and (r['astar']['is_optimal'] or r['astar']['is_optimal'] == None)
                    dijkstra_passed = ('dijkstra' not in r) or (r['dijkstra']['reached_start'] and r['dijkstra']['reached_goal']
                                      and r['dijkstra']['no_collision'] and dijkstra_optimal)
                    astar_passed = ('astar' not in r) or r['astar']['reached_start'] and r['astar']['reached_goal'] and \
                                   r['astar']['no_collision'] and astar_optimal
                    passed = dijkstra_passed and astar_passed
                    print('{} {}, (size {:,})'.format('pass' if passed else 'FAIL', r['test_name'], r['map_nodes']))
                    for name in ['dijkstra', 'astar']:
                        if name in r:
                            print('    {} reached start: {}'.format(name, 'pass' if r[name]['reached_start'] else 'FAIL'))
                            print('    {} reached goal:  {}'.format(name, 'pass' if r[name]['reached_goal'] else 'FAIL'))
                            print('    {} no collision:  {}'.format(name, 'pass' if r[name]['no_collision'] else 'FAIL'))
                            print('    {} is optimal:    {}'.format(name,
                                {True: 'pass', False: 'FAIL', None: '?'}[r[name]['is_optimal']]))
                            print('    {} path length:   {}'.format(name, r[name]['path_length']))
                            print('    {} time, seconds: {}'.format(name, r[name]['time']))
                else:
                    passed = ('dijkstra' not in r or r['dijkstra']['path_length'] == np.inf) and \
                            ('astar' not in r or r['astar']['path_length'] == np.inf)
                    print('{} {}, (size {:,})'.format('pass' if passed else 'FAIL', r['test_name'], r['map_nodes']))
                    for name in ['dijkstra', 'astar']:
                        if name in r:
                            print('    {} is optimal:    {}'.format(name,
                                {True: 'pass', False: 'FAIL', None: '?'}[r[name]['is_optimal']]))
                            print('    {} path length:   {}'.format(name, r[name]['path_length']))
                            print('    {} time, seconds: {}'.format(name, r[name]['time']))
            elif 'error' in r.keys():
                print("FAIL {name}\n"
                      "    {error}".format(name=r['test_name'], error=r['error']))
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
    default_target = 'proj1_2.code'
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