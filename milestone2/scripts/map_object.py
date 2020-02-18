#!/usr/bin/env python

import json

def load(map_path):
    with open(map_path, "r") as f:
        return json.load(f)

class Map:
    def __init__(self, map_path):
        self.map_json = load(map_path)
        
    def 


if __name__ == "__main__":
    pass