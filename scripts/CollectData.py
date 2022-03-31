#!/usr/bin/env python
import zmq 
import numpy as np 
import tf.transformations as tf_utils

import rospy
from tf import TransformListener
import tf.transformations as tf_utils
import message_filters

class CollectData:
    def __init__(self, num_samples:int=15, file_name:str='calib.txt', 
                file_location:str='/home/pairlab/', 
                port:str="tcp://127.0.0.1:2000") -> None:

        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.socket.connect("tcp://127.0.0.1:2000")
        self.num_samples = num_samples
        self.output_file = file_location + file_name

        rospy.init_node('listener', anonymous=True)
        self.tf_listener_ = TransformListener()

        if not rospy.core.is_initialized():
            raise rospy.exceptions.ROSInitException("client code must call rospy.init_node() first")
        try:
            # self.getObservationPoses()
            while not rospy.core.is_shutdown():
                #self.TimeStampPublisher()
                #if not self.data_collected:
                self.getObservationPoses()
                    #self.data_collected = True
                rate.sleep()
        except KeyboardInterrupt:
            rospy.core.signal_shutdown('keyboard interrupt')
            pass
        pass

    def getObservationPoses(self,):
        for i in range(self.num_samples):
            print( i)
            ip = input("Press Enter to continueth sampl3.... else space bar to neglect this sample")
            if (ip==""):
                self.writeObs()
            else:
                print("ignoring input")
                pass          

    def write_to_file(self,line_list):
        # open file in append mode
        line  = ",".join(line_list)
        with open(self.output_file, 'a') as f:
            f.write(line)
            f.write('\n') 

    def writeObs(self,):
        #get calibration target pose wrt camera optical frame
        parent = "camera_color_optical_frame"
        child = "fiducial_0"
        avg_tag_position = np.zeros(3); avg_tag_quaternion = np.zeros(4)
        count = 0
        for k in range(10):
            if self.tf_listener_.frameExists(parent) and self.tf_listener_.frameExists(child):
                t = self.tf_listener_.getLatestCommonTime(parent, child)
                tag_position, tag_quaternion = self.tf_listener_.lookupTransform(parent, child,t)
                avg_tag_position += np.array(tag_position)
                avg_tag_quaternion += np.array(tag_quaternion)
                count = count + 1
            else:
                print(parent, child, " can't find tf")
            
        avg_tag_position = avg_tag_position/count
        avg_tag_quaternion = avg_tag_quaternion/count
            
        
        #get EE pose wrt Base
        EEposition, EEquat = self.getCurrentEEFrame()

        now = rospy.Time.now()
        stamp = [str(i) for i in [now.secs, now.nsecs]]
        calib_tag_pose = [str(i) for i in [avg_tag_position[0], 
                                        avg_tag_position[1], 
                                        avg_tag_position[2], 
                                        avg_tag_quaternion[0], 
                                        avg_tag_quaternion[1], 
                                        avg_tag_quaternion[2], 
                                        avg_tag_quaternion[3]]]
        ee_pose = [str(i) for i in [EEposition[0],
                                EEposition[1],
                                EEposition[2],
                                EEquat[0],
                                EEquat[1],
                                EEquat[2],
                                EEquat[3]]]
        
        line = stamp + calib_tag_pose + ee_pose
        self.write_to_file(line)

    def getCurrentEEFrame(self,):
        self.socket.send_string("data")#even an empty message would do
        message = self.socket.recv()
        zmqPose = np.frombuffer(message).astype(np.float32)
        zmqPose = np.reshape(a=zmqPose, newshape=(4,4), order='F')
        zmq_position = zmqPose[:3,3]
        zmq_quat = tf_utils.quaternion_from_matrix(zmqPose)
        return zmq_position, zmq_quat

    
def main():
    cd_instance = CollectData()

if __name__=="__main__":
    main()