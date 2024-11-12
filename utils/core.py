
def robot_id_to_index(robot_id):
    underscore = robot_id.rindex('_')
    return int(robot_id[underscore + 1:])