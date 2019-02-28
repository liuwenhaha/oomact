#!/bin/env/python

# Roslib gives us an easy way to pull in python libraries
import roslib
# load_manifest() looks for python export tags in the manifest files.
roslib.load_manifest("oomact_backend")

import oomact_backend as oomact
# sm is the Schweizer Messer library
import sm

import unittest
class WheelOdometryCalibrationTests(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
    
    def testWheelOdometryCalibrationStraight(self):
        #load config file somehow
        vs=sm.ValueStoreRef #does this not exist for python yet

        #this does not work yet
        #oomact.FrameGraphModel m(vs.getChild("model"))