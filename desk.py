# E. Culurciello
# March 2021

# multi-task desk environment
# based on: https://github.com/google-research/google-research/tree/master/playrooms

# desk object

import pybullet

DESK_URDF_PATH = "assets/desk/desk.urdf"

class Desk():
    def __init__(self ):
        self.desk = pybullet.loadURDF(DESK_URDF_PATH)

