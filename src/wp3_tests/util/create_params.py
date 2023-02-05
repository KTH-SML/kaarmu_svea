#! /usr/bin/env python3

import yaml
import pathlib
import itertools

if __name__ == '__main__':

    out_dir = pathlib.Path(__file__).parent.parent
    out_dir /= 'params'

    constants = {
        'start_pos': 30,        # [m]
        'target_vel': 10,       # [m/s]
        'delta_time': 0.01,     # [s]
        'state_freq': 30        # [Hz]
    }

    varying = {
        'data_size': list(map(int, [1e0, 1e1, 1e2, 1e3, 1e4])), # [kB]
        'compute_time': [1e-2, 5e-2, 1e-1, 5e-1, 1e0], # [s]
        'compute_order': [0, 1, 2] # Complexity order
    }

    names = varying.keys()
    vs = [list(zip(names, values))
          for values in itertools.product(*varying.values())]

    for i, v in enumerate(vs):

        filename = f'{out_dir}/P{i:03}.yml'
        with open(filename, 'w') as f:
            d = {}
            d.update(constants)
            d.update(v)
            yaml.dump(d, f)
            print(f'Created parameter set: {filename}')

