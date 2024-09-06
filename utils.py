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
