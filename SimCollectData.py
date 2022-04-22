
#from cgi import test
#from frankapy import FrankaArm
#from frankapy.franka_arm_state_client import FrankaArmStateClient
import numpy as np
from scipy.spatial.transform import Rotation as R
import rospy
from franka_msgs.msg import FrankaState
import message_filters
from fiducial_msgs.msg import FiducialTransformArray
from fiducial_msgs.msg import FiducialArray
#from autolab_core import RigidTransform, YamlConfig
from franka_msgs.srv import SetEEFrame, SetEEFrameRequest
#from tf import TransformListener
import tf2_ros
from tf import TransformListener
import tf.transformations as tf_utils

def point_to_object(obj_loc, distance, pan, yaw, rotate):
    r = R.from_rotvec(np.array([0, -pan, 0]))
    r1 = R.from_rotvec(np.array([0,0, yaw]))
    r2 = R.from_rotvec(np.array([0, pan, 0]))
    r3 = R.from_rotvec(np.array([np.pi, 0 ,0]))
    r4 = R.from_rotvec(np.array([0, 0, rotate]))
    translation = r1.as_matrix() @ r.as_matrix() @ np.array([distance, 0, 0])
    translation += obj_loc
    return translation, r1.as_matrix() @ r2.as_matrix() @ r3.as_matrix() @ r4.as_matrix()

class Eye2HandCalibration:
    def __init__(self, is_sim=False, file_name:str='calib.txt', file_location:str='/home/pairlab/') -> None:
        self.is_sim = is_sim
        self.output_file = file_location + file_name
        self.current_franka_state = None
        self.current_transform = None
        self.current_aurco_img_vertices = None
        self.is_new = False
        self.data_collected = False
        rospy.init_node('listener', anonymous=True)
        self.tf_listener_ = TransformListener()
        #rospy.Subscriber("chatter", FrankaState, self.frankaStateCallBack)
        if self.is_sim:
            self.aruco_transforms_sub = message_filters.Subscriber("/fiducial_transforms", FiducialTransformArray, queue_size=3)
            self.aruco_img_vertices_sub = message_filters.Subscriber("/fiducial_vertices", FiducialArray, queue_size=3)
            self.franka_states_sub = message_filters.Subscriber("/franka_state_controller/franka_states", FrankaState, queue_size=3)
        else: 
            self.aruco_transforms_sub = message_filters.Subscriber("/fiducial_transforms", FiducialTransformArray, queue_size=3)
            self.aruco_img_vertices_sub = message_filters.Subscriber("/fiducial_vertices", FiducialArray, queue_size=3)
            self.franka_states_sub = message_filters.Subscriber("/franka_state_controller/franka_states", FrankaState, queue_size=3)

        self.ts = message_filters.ApproximateTimeSynchronizer([self.franka_states_sub, self.aruco_transforms_sub, self.aruco_img_vertices_sub], 10,2.0, allow_headerless=True) 
        self.ts.registerCallback(self.SynchCallback)
        self.setEEframeservice = rospy.ServiceProxy('/set_EE_frame', SetEEFrame)
        rate = rospy.Rate(20) # 30hz

        if not rospy.core.is_initialized():
            raise rospy.exceptions.ROSInitException("client code must call rospy.init_node() first")
        try:
            # self.getObservationPoses()
            while not rospy.core.is_shutdown():
                #self.TimeStampPublisher()
                if not self.data_collected:
                    self.getObservationPoses()
                    self.data_collected = True
                rate.sleep()
        except KeyboardInterrupt:
            rospy.core.signal_shutdown('keyboard interrupt')
            pass
     

        

    def SynchCallback(self,franka_state_msg:FrankaState, aruco_trans_msg:FiducialTransformArray, aruco_img_vertices_msg:FiducialArray):
        self.current_franka_state = franka_state_msg
        self.current_transform = aruco_trans_msg
        self.current_aurco_img_vertices = aruco_img_vertices_msg
        self.is_new = True
        parent = "camera_link_optical"
        child = "camera_link_optical"
        #if self.tf_listener_.frameExists(parent) and self.tf_listener_.frameExists(child):
            #t = self.tf_listener_.getLatestCommonTime(parent, child)
            #tag_position, tag_quaternion = self.tf_listener_.lookupTransform(parent,child,rospy.Time(0.0))
            #print('tf tag', tag_position, tag_quaternion)
        #try:    
            #print('aruco', aruco_trans_msg.transforms[0].transform.translation, aruco_trans_msg.transforms[0].transform.rotation)
        #    ee_transform = franka_state_msg.O_T_EE
        #    ee_transform = np.asarray(ee_transform).reshape(4, 4, order='F')
        #    ee_quat = tf_utils.quaternion_from_matrix(ee_transform[:, :])
            #print('aruco', [ee_transform[0,3], ee_transform[1,3], ee_transform[2,3]], ee_quat)
        #except:
        #    pass
        #print('synchinf')

        pass

    def getObservationPoses(self,):
        pan = np.pi / 4
        location = np.array([0.8, 0, 0])
        for i in range(20):
            distance = 0.3 + 0.05 * i // 3
            yaw = np.pi * (0.7 + 0.07 * i)
            if self.is_sim:
                pass
            else:
                # endfactor_pose = RigidTransform()                    
                # endfactor_pose.translation, endfactor_pose.rotation =  point_to_object(location, distance, pan, yaw, 0)
                # ee_frame = np.concatenate((np.array(endfactor_pose.rotation), np.array(endfactor_pose.translation).reshape(3,1)), axis=1  )
                # ee_frame = np.vstack((ee_frame, np.array([0.0, 0.0, 0.0, 1.0])))
                # ee_frame = np.reshape(ee_frame,(1,16),order='f') #convert transformation matrix to list in column-major format
                # print(ee_frame)
                # req = SetEEFrameRequest()
                # req.NE_T_EE = list(ee_frame[0])
                # resp = self.setEEframeservice(req)
                # print(resp)
                ip = input("Press Enter to continue... else space bar to neglect this sample")
                if (ip==""):
                    self.write_msgs(aruco_trans_msg=self.current_transform, fiducial_det_msg=self.current_aurco_img_vertices, franka_state_msg=self.current_franka_state)
                elif(ip==" "):
                    print("ignoring it")
                    pass
                else:
                    print("ignoring it")
                    pass
        exit()

    def write_to_file(self,line_list):
        # open file in append mode
        line  = ",".join(line_list)
        with open(self.output_file, 'a') as f:
            f.write(line)
            f.write('\n')
    
    def write_msgs(self,aruco_trans_msg, fiducial_det_msg, franka_state_msg):

        if self.is_sim:
            parent = "camera_link_optical"
            child = "DICT_4X4_50_id0_link_0"
            if self.tf_listener_.frameExists(parent) and self.tf_listener_.frameExists(child):
                t = self.tf_listener_.getLatestCommonTime(parent, child)
                gttag_position, gttag_quaternion = self.tf_listener_.lookupTransform(parent, child,t)
            parent = "fiducial_1"
            child = "DICT_4X4_50_id0_link_0"
            if self.tf_listener_.frameExists(parent) and self.tf_listener_.frameExists(child):
                t = self.tf_listener_.getLatestCommonTime(child, parent)
                test_position, test_quaternion = self.tf_listener_.lookupTransform(parent, child,t)

            parent = "camera_link_optical"
            child = "fiducial_1"
            if self.tf_listener_.frameExists(parent) and self.tf_listener_.frameExists(child):
                t = self.tf_listener_.getLatestCommonTime(parent, child)
                tag_position, tag_quaternion = self.tf_listener_.lookupTransform(parent, child,t)
            parent = "panda_link0"
            child = "panda_NE"
            if self.tf_listener_.frameExists(parent) and self.tf_listener_.frameExists(child):
                t = self.tf_listener_.getLatestCommonTime(parent, child)
                ee_position, ee_quaternion = self.tf_listener_.lookupTransform(parent, child,t)       
        else: 
            parent = "camera_color_optical_frame"
            child = "fiducial_0"
            if self.tf_listener_.frameExists(parent) and self.tf_listener_.frameExists(child):
                t = self.tf_listener_.getLatestCommonTime(parent, child)
                tag_position, tag_quaternion = self.tf_listener_.lookupTransform(parent, child,t)
            else:
                print(parent, child, " can't find tf")

            parent = "panda_link0"
            child = "panda_NE"
            now = rospy.Time.now()
            self.tf_listener_.waitForTransform(parent, child, now, rospy.Duration(5000.0))
            print('waited')
            #if self.tf_listener_.frameExists(parent) and self.tf_listener_.frameExists(child):
            t = self.tf_listener_.getLatestCommonTime(parent, child)
            ee_position, ee_quaternion = self.tf_listener_.lookupTransform(parent, child,t)   
            print(ee_position, ee_quaternion)
            #else:
            #    print(parent, child, " can't find tf")            
        

        if(len(aruco_trans_msg.transforms) != 0 and len(fiducial_det_msg.fiducials)!= 0):
            stamp = [str(i) for i in [aruco_trans_msg.header.stamp.secs, aruco_trans_msg.header.stamp.nsecs]]
            fiducial = [str(i) for i in[tag_position[0], 
                                        tag_position[1], 
                                        tag_position[2], 
                                        tag_quaternion[0], 
                                        tag_quaternion[1], 
                                        tag_quaternion[2], 
                                        tag_quaternion[3],
                                        aruco_trans_msg.transforms[0].image_error,
                                        aruco_trans_msg.transforms[0].object_error,
                                        aruco_trans_msg.transforms[0].fiducial_area,]]


            fiducial_img_det = [str(i) for i in [fiducial_det_msg.fiducials[0].x0, fiducial_det_msg.fiducials[0].y0,
                                                fiducial_det_msg.fiducials[0].x1, fiducial_det_msg.fiducials[0].y1,
                                                fiducial_det_msg.fiducials[0].x2, fiducial_det_msg.fiducials[0].y3,
                                                fiducial_det_msg.fiducials[0].x3, fiducial_det_msg.fiducials[0].y3,]]

            
            ee_transform = franka_state_msg.O_T_EE
            ee_transform = np.asarray(ee_transform).reshape(4, 4, order='F')
            ee_quat = tf_utils.quaternion_from_matrix(ee_transform[:, :])#only rotation matrix
            if(franka_state_msg.O_T_EE[12]!= ee_transform[0,3] and 
                franka_state_msg.O_T_EE[13]!= ee_transform[1,3] and 
                franka_state_msg.O_T_EE[13]!= ee_transform[2,3]):
                rospy.loginfo("something wrong")
                exit()
            ee_frame = [str(i) for i in [ee_position[0], ee_position[1], ee_position[2], 
                                        ee_quaternion[0], ee_quaternion[1], ee_quaternion[2], ee_quaternion[3]]]

            j_angles_ = [str(i) for i in franka_state_msg.q]

            if self.is_sim:
                gt_tag = [str(i) for i in [gttag_position[0], gttag_position[1], gttag_position[2],
                                            gttag_quaternion[0], gttag_quaternion[1], gttag_quaternion[2],gttag_quaternion[3]]] 
                test_tag = [str(i) for i in [test_position[0], test_position[1], test_position[2],
                                            test_quaternion[0], test_quaternion[1], test_quaternion[2],test_quaternion[3]]]             
                line = stamp + fiducial + fiducial_img_det + ee_frame + j_angles_ + gt_tag + test_tag
            else: 
                line = stamp + fiducial + fiducial_img_det + ee_frame + j_angles_

            self.write_to_file(line)




def main():
    obj = Eye2HandCalibration()

if __name__ == "__main__":
    main() 
