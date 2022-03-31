# https://github.com/fsuarez6/opencv-handeye/blob/master/main.cp
# 
# https://www.geeksforgeeks.org/python-all-possible-pairs-in-list/
from typing import Optional
import numpy as np 
import tf.transformations as tf_utils
import timeit
import park_martin
from solvers import Daniilidis1999, ParkBryan1994, TsaiLenz1989
from eye2handRANSAC import Eye2HandRANSAC as e2hrs

class Calibrate:
    def __init__(self, data_file_name, method=None) -> None:
        self.marker_cam_rot = []
        self.marker_cam_trans = []
        self.hand_base_rot = []
        self.hand_base_trans = []
        self.data = []
        self.A = []
        self.B = []
        self.solvers = { "daniilidis": Daniilidis1999,
                        "parkbryan": ParkBryan1994,
                        "tsaiLenz": TsaiLenz1989,
                        }
        self.true_rot = np.array([-90.0, 0.0, 0.0])
        self.true_trans = np.array([0.0, -1.0, 0.5])

        with open(data_file_name, 'r') as fp:
            lines = fp.readlines()
        for line in lines:
            data = line.split('\n')[0].split(',')

            marker_in_cam_trans = np.array((float(data[2]), float(data[3]), float(data[4])))
            marker_in_cam = tf_utils.quaternion_matrix([float(data[5]), float(data[6]), float(data[7]), float(data[8])])
            marker_in_cam[0:3,3] = marker_in_cam_trans

            # ee_in_base_trans = np.array((float(data[20]), float(data[21]), float(data[22])))
            # ee_in_base = tf_utils.quaternion_matrix([float(data[23]), float(data[24]), float(data[25]), float(data[26])])
            ee_in_base_trans = np.array((float(data[9]), float(data[10]), float(data[11])))
            ee_in_base = tf_utils.quaternion_matrix([float(data[12]), float(data[13]), float(data[14]), float(data[15])])
            
            ee_in_base[0:3,3] = ee_in_base_trans


            self.data.append(tuple((marker_in_cam, ee_in_base)))
        indices = range(len(self.data))
        pairs = [(a, b) for idx, a in enumerate(indices) for b in indices[idx + 1:]]
        for pair in pairs:
            marker_in_cam1 = self.data[pair[0]][0]; ee_in_base1 = self.data[pair[0]][1]
            marker_in_cam2 = self.data[pair[1]][0]; ee_in_base2 = self.data[pair[1]][1] 

            A = np.matmul( ee_in_base1, park_martin.matinv(ee_in_base2))
            B = np.matmul(marker_in_cam1, park_martin.matinv(marker_in_cam2)) 

            self.A.append(A)
            self.B.append(B)
        print("nuber of samples= ", len(self.A))
        for key,value in self.solvers.items():
            print("algorithm: ", key)
            start = timeit.default_timer()
            solver_instance = value()
            ransac_obj = e2hrs(As=self.A, Bs=self.B, solver=solver_instance)
            Xhat = ransac_obj.Run()
            #angles = tf_utils.euler_from_matrix(Xhat)
            Xhat = tf_utils.inverse_matrix(Xhat)
            angles = tf_utils.quaternion_from_matrix(Xhat)
            angles = np.array(angles)#*180.0/np.pi
            print("rotation:", angles)
            print("translation= ", Xhat[0:3,3] )
            # print('trans_err= ', abs(Xhat[0:3,3] - self.true_trans))
            # print('rot_err= ', abs(angles - self.true_rot))
            stop = timeit.default_timer()
            print('run time= ', stop - start)

            print("Without RANSAC algorithm: ", key)
            start = timeit.default_timer()
            solver_instance = value()
            Xhat = solver_instance(self.A, self.B)
            # angles = tf_utils.euler_from_matrix(Xhat)
            Xhat = tf_utils.inverse_matrix(Xhat)
            angles = tf_utils.quaternion_from_matrix(Xhat)
            angles = np.array(angles)#*180.0/np.pi
            print("rotation:", angles)
            print("translation= ", Xhat[0:3,3] )
            # print('trans_err= ', abs(Xhat[0:3,3] - self.true_trans))
            # print('rot_err= ', abs(angles - self.true_rot))
            stop = timeit.default_timer()
            print('run time= ', stop - start)


def main(file_):
    cb_obj = Calibrate(file_)
    pass


if __name__ == '__main__':
    file_ = "/home/pairlab/calib.txt"
    main(file_)


