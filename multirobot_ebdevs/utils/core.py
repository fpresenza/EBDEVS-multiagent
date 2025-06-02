#!/usr/bin/env python
# -*- coding: utf-8 -*-

def robot_id_to_index(robot_id):
    underscore = robot_id.rindex('_')
    return int(robot_id[underscore + 1:])


def target_id_to_index(target_id):
    underscore = target_id.rindex('_')
    return int(target_id[underscore + 1:])
