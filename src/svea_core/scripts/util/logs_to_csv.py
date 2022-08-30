import os
import glob
import pickle
import pandas as pd

import matplotlib.pyplot as plt
import matplotlib as mpl
mpl.style.use('seaborn-muted')

dirname = os.path.dirname(__file__)
LOG_DIR = os.path.join(dirname, '../../resources/logs/')
SAVE_PREFIX = 'svea_data'

def save_as_csv(log_filename, idx):
    pkl_file = open(log_filename, 'r')

    data_dict = pickle.load(pkl_file)

    ctrl_name_map = {'ctrl_t': 't (sec)',
                    'ctrl_steer': 'steering angle (rad)',
                    'ctrl_v': 'velocity ctrl (m/s)',
                    'ctrl_trans': 'transmission (0=Low or 1=High)'}

    state_name_map = {'t': 't (sec)',
                    'x': 'x (m)',
                    'y': 'y (m)',
                    'yaw': 'heading (rad)',
                    'v': 'velocity state (m/s)'}

    ctrl_dict = {ctrl_name_map[key]: data_dict[key]
                for key in ctrl_name_map.keys()}
    state_dict = {state_name_map[key]: data_dict[key]
                for key in state_name_map.keys()}

    ctrl_dataframe = pd.DataFrame.from_dict(ctrl_dict)
    state_dataframe = pd.DataFrame.from_dict(state_dict)

    ctrl_dataframe.to_csv(LOG_DIR + SAVE_PREFIX + '_input' + str(idx) + '.csv')
    state_dataframe.to_csv(LOG_DIR + SAVE_PREFIX + '_state' + str(idx) + '.csv')

for i, log_filename in enumerate(glob.glob(LOG_DIR+"*.pkl")):
    save_as_csv(log_filename, i)
