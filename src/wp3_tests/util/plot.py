#! /usr/bin/env python3

"""
This script will generate the figures described below from a given test-suite run.

Figure 1: Latency vs Packet Size trade-off

    This figure will have axes Latency and Packet Size. The diagram is
    split into one "safe" and one "unsafe" area. The latter is the
    more important. When in this region it is not possible for the vehicles
    to satisfy the safety criteria at all. In general we see the trade-off
    between latency and Packet Size. From this we say something about the
    impact throughput (i.e. amount of data we can push over some time) has
    on safety.
"""

import json
from argparse import ArgumentParser
from pathlib import Path
from typing import Any, List

import matplotlib.pyplot as plt

LOG_FIELD_SEP = ','
LOG_ENTRY_SEP = ';'

def load(dir: Path):
    confs, results = {}, {}

    for test_dir in dir.iterdir():
        if not test_dir.is_dir():
            continue
        for file in test_dir.iterdir():
            with open(file) as f:
                if file.name == 'conf':
                    # {
                    #   'INIT_POS': ...,
                    #   'TARG_VEL': ...,
                    #   'TIME_STEP': ...,
                    #   'DATA_SIZE': ...,
                    #   'DATA_FREQ': ...,
                    #   'COMP_TIME': ...,
                    #   'COMP_ORDR': ...,
                    # }
                    confs[test_dir.name] = json.load(f)

                else:
                    if test_dir.name not in results:
                        results[test_dir.name] = []

                    # [[sender, sent, arrival, valid, headway, safe], ...]
                    result = parse_content(f.read())
                    results[test_dir.name].append((file.name, result))

    return confs, results

def parse_content(content: str) -> List[List]:
    return [
        [parse_field(field) for field in entry.split(LOG_FIELD_SEP)]
        for entry in content.split(LOG_ENTRY_SEP)
    ]

def parse_field(field: str):
    if field.isnumeric():
        return int(field)
    elif field.replace('.', '', 1).isnumeric():
        return float(field)
    elif field == 'True':
        return True
    elif field == 'False':
        return False
    else:
        return field

def generate_fig1(confs, results, out_dir):
    # confs = { test_name: {...} }
    # results = { test_name: [(veh_name, result), ...] }

    fig: Any = plt.figure()
    ax: Any = fig.gca()

    ax.set_title('Latency vs Packet Size trade-off')
    ax.set_xlabel('Packet Size [kB]')
    ax.set_ylabel('Latency [ms]')

    red = []
    green = []

    for test_name, conf in confs.items():
        for veh_name, result in results[test_name]:
            if not veh_name == 'svea5': continue
            # result = [[sender, sent, arrival, valid, headway, safe], ...]
            for entry in result:
                try:
                    packet_size = conf['DATA_SIZE']
                    arrival = entry[2]  # [ns] saved as ros time
                    sent = entry[1]     # [ns]
                    safe = entry[-1]

                    latency = arrival - sent    # [ns]
                    latency /= 1e6              # [ms]

                    # x-axis, y-axis
                    point = (packet_size, latency)

                    bin = green if safe else red
                    bin.append(point)
                except Exception as e:
                    print(f'{type(e).__name__} ({test_name}: {veh_name}):', str(e))
                    return

    red_xs, red_ys = zip(*red)
    green_xs, green_ys = zip(*green)

    ax.scatter(green_xs, green_ys, c='green')
    ax.scatter(red_xs, red_ys, c='red')

    filename = out_dir / 'fig1.png'
    fig.savefig(filename)
    print(f'Generated Figure 1 at {filename}')

if __name__ == '__main__':

    ## Arguments

    parser = ArgumentParser()

    parser.add_argument('dir', help='Directory to log data (for one test-suite run)')

    args = parser.parse_args()

    ## Parse data

    dir = Path() / args.dir
    print(f'Chosen directory: {dir}')

    confs, results = load(dir)

    generate_fig1(confs, results, dir)

