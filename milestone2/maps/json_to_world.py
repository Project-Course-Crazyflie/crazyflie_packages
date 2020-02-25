#!/usr/bin/env python

"""
    @author: Daniel Duberg (dduberg@kth.se)
"""

from __future__ import print_function, unicode_literals
import rospkg
import json
import sys
import os
import re
import gates_to_walls
import generate_gazebo_world
import glob


def how_to_use(json_path, default_physics_iterations):
    file_type_length = len('.json')
    print('')
    print(
        'USAGE: rosrun dd2419_simulation json_to_world.py [OPTION [OPTION ...]] [FILENAME]')
    print('')
    print('Where OPTION is one of:')
    print('    -a, --all                            Process all JSON files found')
    print('    -h, --help                           Displays this usage text')
    print('    -i, --iterations NUM_ITERATIONS      Where NUM_ITERATIONS is the number of iterations for the physics engine. Default: ' +
          str(default_physics_iterations))
    print('')
    print('Where FILENAME is one of:')
    for file in sorted(os.listdir(json_path)):
        if len(file) > file_type_length and file[-file_type_length:] == '.json':
            print("    * ", re.sub(re.escape('.json') + '$', '', file))
    print('')


def main(argv):
    rospack = rospkg.RosPack()
    # get the file path for dd2419_simulation
    sim_pkg_path = rospack.get_path('milestone2/sripts')
    # get the file path for dd2419_resources
    resource_pkg_path = rospack.get_path('milestone2/sripts')

    json_path = resource_pkg_path + '/maps/'

    process_all = False
    physics_iterations = 500

    if len(argv) == 0:
        how_to_use(json_path, physics_iterations)
        exit(1)

    last_is_used = True
    for index in range(0, len(argv)):
        arg = argv[index]
        if arg == '-a' or arg == '--all':
            process_all = True
        elif arg == 'h' or arg == '--help':
            how_to_use(json_path)
            exit(0)
        elif arg == '-i' or arg == '--iterations':
            index = index + 1
            if index == len(argv):
                print('You have to input how many iterations after: "' + arg + '"')
            else:
                physics_iterations = int(argv[index])
        elif index == len(argv) - 1:
            last_is_used = False

    if last_is_used and not process_all:
        how_to_use(json_path)
        exit(2)

    process_files = []

    if process_all:
        process_files = glob.glob(json_path + '*.json')
    elif not last_is_used:
        process_files = [json_path + argv[len(argv)-1]]

    print("")
    print("")
    for json_file in process_files:
        filename = re.sub('\.json$', '', os.path.basename(json_file))
        print("Processing: " + filename)
        print("")
        print("")
        with open(re.sub('\.json$', '', json_file) + ".json") as f:
            data = json.load(f)

            print("Turning gates into walls")
            # 'date =' is not necessary, but easier to read
            data = gates_to_walls.gates_to_walls(data)
            print("")

            # save_path = json_path + 'walled/'
            # if not os.path.exists(save_path):
            #     os.makedirs(save_path)
            # save_path = save_path + 'walled_' + filename + '.json'
            # print("Saving walled json world to:", save_path)
            # with open(save_path, 'w') as f:
            #     json.dump(data, f, default=list, indent=2)
            # print("")

            print("Generating world")
            if not os.path.exists(sim_pkg_path + '/maps/'):
                os.makedirs(sim_pkg_path + '/maps/')
            generate_gazebo_world.generate_world(
                sim_pkg_path + '/maps/', re.sub('\.world$', '', filename) + '.world', data, sim_pkg_path, physics_iterations)
            print("")
        print("")
        print("")
        print("Done with: " + filename)
        print("")
        print("")


if __name__ == "__main__":
    main(sys.argv[1:])
