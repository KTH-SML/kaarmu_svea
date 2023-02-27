import os
from os import path
import json
import pandas as pd

LOG_DIR = os.path.abspath("../log")


def load_csv(
    csv_path: str,
    conf: dict,
    headers=[
        "TARGET_ID",
        "TIME_SENT",
        "TIME_ARRIVED",
        "COUNT",
        "IS_VALID",
        "POSITION",
        "IS_SAFE",
    ],
) -> pd.DataFrame:
    """Reads csv file and adds headers as column names. Returns a data frame

    Args:
        csv_path (str):
        headers (list of str): column names

    Returns:
        pd.DataFrame: _description_
    """
    df = pd.read_csv(csv_path, sep=",", names=headers, lineterminator=";")
    df["LATENCY"] = (df["TIME_ARRIVED"] - df["TIME_SENT"]) / 1e6  # ms
    df["THROUGHPUT"] = conf["DATA_SIZE"] * conf["DATA_FREQ"]  # kB/s
    return df


def load_json(conf_dir):
    with open(conf_dir, "r") as f:
        conf = json.loads(f.read())
    return conf


def load_test_case(test_suite_dir: str, test_case_dir: str):
    # header = f"OPENING {test_case_dir} from {test_suite_dir}"
    # header = f"{header}\n{'_'*len(header)}"
    # print(header)
    case_path = path.join(LOG_DIR, test_suite_dir, test_case_dir)
    if not case_path.endswith("/"):
        case_path = case_path + "/"

    with open(case_path + "conf", "r") as f:
        conf = json.loads(f.read())
    conf = load_json(case_path + "conf")
    svea2_df = load_csv(case_path + "svea2", conf)
    svea5_df = load_csv(case_path + "svea5", conf)

    return conf, svea2_df, svea5_df


def load_all_conf(test_suite_dir: str):
    all_test_cases = [
        file
        for file in os.listdir(path.join(LOG_DIR, test_suite_dir))
        if "test" in file
    ]

    df = pd.DataFrame()
    for test_case in all_test_cases:
        conf = load_json(path.join(LOG_DIR, test_suite_dir, test_case, "conf"))
        conf["THROUGHPUT"] = conf["DATA_SIZE"] * conf["DATA_FREQ"]
        index = int(test_case.split("_")[-1])
        df = pd.concat([df, pd.DataFrame(conf, index=[index])])

    return df.sort_index(axis=0)


def get_good_runs():
    """Returns a list of all "good" experiments defined as all the runs which
    have a corresponding .tar.gz file

    Returns:
        (list of str): List of the directory names for the "good" experiments
    """
    return [
        file.split(".")[0]
        for file in os.listdir(path=os.path.abspath("../log/"))
        if "tar.gz" in file
    ]


if __name__ == "__main__":
    # good_runs = get_good_runs()
    conf, svea2_df, svea5_df = load_test_case("1677152050268280982", "test_1")
    print(load_all_conf("1677152050268280982").head(5))
