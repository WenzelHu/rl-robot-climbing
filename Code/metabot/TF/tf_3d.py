import numpy as np

class tf_3d:
    def __init__(self):# local coordinate system origin in global coordinate system
        self.x_offset = None
        self.y_offset = None
        self.z_offset = None
        self.alpha = None
        self.beta = None
        self.gamma = None
        self.trans_matrix = None
        self.inv_trans_matrix = None
        
    def set_tf(self, x, y, z, alpha, beta, gamma):
        self.x_offset = x
        self.y_offset = y
        self.z_offset = z
        self.alpha = alpha
        self.beta = beta
        self.gamma = gamma
        self.trans_matrix = np.matrix(np.zeros([4,4]))
        self.trans_matrix[0:3, 0:3] = self.angle_to_matrix(alpha, beta, gamma)
        self.trans_matrix[0:4, 3] = np.matrix([x, y, z, 1]).T
        self.inv_trans_matrix = self.trans_matrix.I
        
    def set_tf_matrix(self, trans_matrix):
        if trans_matrix is not None:
            self.trans_matrix = np.asmatrix(trans_matrix)
            self.inv_trans_matrix = self.trans_matrix.I
            self.x_offset = self.trans_matrix[0,3]
            self.y_offset = self.trans_matrix[1,3]
            self.z_offset = self.trans_matrix[2,3]
            self.alpha, self.beta, self.gamma = self.matrix_to_angle(self.trans_matrix)
        
    def angle_to_matrix(self, alpha, beta, gamma):
        alpha_matrix = np.matrix([[1, 0, 0], 
                                 [0, np.cos(alpha), -np.sin(alpha)], 
                                 [0, np.sin(alpha), np.cos(alpha)]])
                                 
        beta_matrix = np.matrix([[np.cos(beta), 0, np.sin(beta)], 
                                [0, 1, 0], 
                                [-np.sin(beta), 0, np.cos(beta)]])
                                
        gamma_matrix = np.matrix([[np.cos(gamma), -np.sin(gamma), 0], 
                                 [np.sin(gamma), np.cos(gamma), 0], 
                                 [0, 0, 1]])
                                 
        rotate_matrix = alpha_matrix*beta_matrix*gamma_matrix
        return rotate_matrix
        
    def matrix_to_angle(self, matrix): # not unique
        alpha = np.arctan2(-matrix[1,2], matrix[2,2])
        beta = np.arcsin(matrix[0,2])
        gamma = np.arctan2(-matrix[0,1], matrix[0,0])
        return alpha, beta, gamma
        
    def global_to_local(self, x, y, z):
        if self.trans_matrix is None:
            print("[ERROR] Use set_tf(x, y, theta) before Transform")
            return None, None, None
        global_coor = np.matrix([x, y, z, 1]).T
        local_coor = self.inv_trans_matrix * global_coor
        return local_coor[0,0], local_coor[1,0], local_coor[2,0]
    
    def local_to_global(self, x, y, z):
        if self.trans_matrix is None:
            print("[ERROR] Use set_tf(x, y, theta) before Transform")
            return None, None, None
        local_coor = np.matrix([x, y, z, 1]).T
        global_coor = self.trans_matrix * local_coor
        return global_coor[0,0], global_coor[1,0], local_coor[2,0]
