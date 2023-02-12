"""
Simple test maps should created by directly editing the .json text file,
following any of the existing examples.

This script directly generates some test scenario files and saves them as .json
files. It works, but this is probably not what you want to do.
"""

from flightsim.world import World
from proj1_2.code_soln.occupancy_map import OccupancyMap

if __name__ == "__main__":

    # A grid of trees.
    world = World.grid_forest(n_rows=4, n_cols=3, width=0.5, height=3.0, spacing=2.0)
    world.world['start'] = (1, 1, 1)
    world.world['goal'] = (3, 6, 2)
    world.world['resolution'] = (0.5, 0.5, 0.5)
    world.world['margin'] = 0.1
    world.to_file('example_grid_forest.json')

    # Some random trees.
    world = World.random_forest(world_dims=(5, 5, 3), tree_width=0.2, tree_height=3.0, num_trees=10)
    world.world['start'] = (0.5, 0.5, 0.5)
    world.world['goal'] = (3, 4, 2.5)
    world.world['resolution'] = (0.5, 0.5, 0.5)
    world.world['margin'] = 0.1
    world.to_file('example_random_forest.json')
