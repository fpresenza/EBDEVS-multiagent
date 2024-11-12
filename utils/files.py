import os
import csv
import json


def read_json_file(filename):
    with open(filename, 'r') as file:
        config_dict = json.load(file)
    return config_dict


def write_json_file(filename, data):
    with open(filename, 'w') as file:
        json.dump(data, file, indent=4)


def append_csv_file(filename, data):
    with open(filename, 'a', newline='') as csvfile:
        writer = csv.writer(csvfile, delimiter=',')
        writer.writerow(data)


def find_latest_timestamp(parent_path):
    """Finds the latest timestamp amongst a set of directories
    located in parent_path, and named with format:
        "%Y-%m-%d_%H-%M-%S"
    """
    directories = [d for d in os.listdir(parent_path) if os.path.isdir(parent_path + d)]
    latest = directories[0]
    
    for timestamp in directories[1:]:
        if timestamp > latest:
            latest = timestamp

    return parent_path + latest + '/'