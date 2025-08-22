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


def read_jsonl_file(filename):
    with open(filename) as f:
        data = [json.loads(line) for line in f]
    return data


def append_jsonl_file(filename, data):
    with open(filename, 'a') as f:
        json.dump(data, f)
        f.write('\n')


def read_csv_file(filename):
    data = []
    with open(filename, newline='') as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            data.append(row)
    return data


def write_csv_file(filename, data):
    with open(filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile, delimiter=',')
        for row in data:
            writer.writerow(row)


def append_csv_file(filename, data):
    with open(filename, 'a', newline='') as csvfile:
        writer = csv.writer(csvfile, delimiter=',')
        writer.writerow(data)


def find_latest_timestamp(parent_path):
    """Finds the latest timestamp amongst a set of directories
    located in parent_path, and named with format:
        "%Y-%m-%d_%H-%M-%S"
    """
    directories = [
        d for d in os.listdir(parent_path) if os.path.isdir(parent_path + d)
    ]
    latest = directories[0]

    for timestamp in directories[1:]:
        if timestamp > latest:
            latest = timestamp

    return parent_path + latest + '/'
