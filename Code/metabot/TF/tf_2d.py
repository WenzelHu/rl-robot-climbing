import numpy as np

class tf_2d:
    def __init__(self):# local coordinate system origin in global coordinate system
        self.x_offset = None
        self.y_offset = None
        self.theta = None
        self.trans_matrix = None
        self.inv_trans_matrix = None
        
    def set_tf(self, x, y, theta):
        self.x_offset = x
        self.y_offset = y
        self.theta = theta
        self.trans_matrix = np.matrix([[np.cos(self.theta), -np.sin(self.theta), self.x_offset], [np.sin(self.theta), np.cos(self.theta), self.y_offset], [0, 0, 1]])
        self.inv_trans_matrix = self.trans_matrix.I
        
    def global_to_local(self, x, y):
        if self.trans_matrix is None:
            print("[ERROR] Use set_tf(x, y, theta) before Transform")
            return None, None
        global_coor = np.matrix([x, y, 1]).T
        local_coor = self.inv_trans_matrix * global_coor
        return local_coor[0,0], local_coor[1,0]
    
    def local_to_global(self, x, y):
        if self.trans_matrix is None:
            print("[ERROR] Use set_tf(x, y, theta) before Transform")
            return None, None
        local_coor = np.matrix([x, y, 1]).T
        global_coor = self.trans_matrix * local_coor
        return global_coor[0,0], global_coor[1,0]
