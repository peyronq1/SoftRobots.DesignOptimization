# -*- coding: utf-8 -*-
"""Config for the SensorFinger"""

__authors__ = "tnavez"
__contact__ = "tanguy.navez@inria.fr"
__version__ = "1.0.0"
__copyright__ = "(c) 2020, Inria"
__date__ = "Oct 28 2022"


import sys
import pathlib
sys.path.insert(0, str(pathlib.Path(__file__).parent.absolute())+"/../")
sys.path.insert(0, str(pathlib.Path(__file__).parent.absolute()))

from BaseConfig import GmshDesignOptimization

import numpy as np 

class Config(GmshDesignOptimization):
    def __init__(self):
        super(GmshDesignOptimization,self).__init__("ApexSensor")
        
    def init_model_parameters(self):

        ########################
        ### Parametric Model ###
        ########################

        # Elasticity parameters
        self.PoissonRation = 0.45 #0.47
        self.YoungsModulus = 2.0 # in MPa
        
    def get_design_variables(self):            
        return {"YoungModulus": [self.YoungsModulus, 0.002, 20],
        "PoissonRation": [self.PoissonRation, 0.3, 0.49],
        }

    def get_objective_data(self):
        return {"ErrorForceDisp": ["minimize", 100],
        "ErrorForcePressure":["minimize", 100]}

    def get_assessed_together_objectives(self):
        return [["ErrorForceDisp", "ErrorForcePressure"]]

    def set_design_variables(self, new_values):
        super(Config,self).set_design_variables(new_values)


    
