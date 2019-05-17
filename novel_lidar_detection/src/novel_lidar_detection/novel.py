import rospy
from novel_msgs.msg import NovelObject, NovelObjectArray
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
import numpy as np
import math
from copy import deepcopy
class NovelLidarDetection(object):
    def __init__(self, 
        in_scan_topic='/scan', 
        expected_scan_topic='/expected_scan', 
        out_scan_topic='filtered_scan', 
        detected_object_topic='/lidar_objects', 
        amcl_pose_topic='/amcl_pose',
        odom_topic='/odom',
        threshold=0.05, 
        window_size=3, 
        window_step=2,
        covariance_threshold=0.007,
        frame_id='base_scan',
        size_threshold=0.1):
        """
        Initializes object 

        params
        ------
        in_scan_topic : str
            Real scan topic
        expected_scan_topic : str
            Expected scan topic
        out_scan_topic : str
            Filtered scan topic to go to localization nodes
        detected_object_topic : str
            Topic to publish detected objects to
        pose_topic : str
            Topic to subscribe to pose
        threshold: float
            Error threshold from cross correlation to count as a new object
        window_size: int
            Sliding window size. Window is slid across both signals, getting an cross correlation error
        window_step: int
            Window step size.
        covariance_threshold: float
            Any pose with a covariance higher than this threshold will cause the real scan to be passed through
            and no objects to be detected.
        frame_id: str
            Name of the tf frame of the scanner (for publishing distance)
        size_threshold: float
            Objects smaller than this size will not be published
        """
        rospy.Subscriber(in_scan_topic, LaserScan, self.ls_callback)
        rospy.Subscriber(expected_scan_topic, LaserScan, self.els_callback)
        rospy.Subscriber(amcl_pose_topic, PoseStamped, self.pose_callback)
        rospy.Subscriber(odom_topic, Odometry, self.odom_callback)
        self.out_scan_pub = rospy.Publisher(out_scan_topic, LaserScan, queue_size=5)
        self.detected_object_pub = rospy.Publisher(detected_object_topic, NovelObjectArray, queue_size=5)
        self.last_scan =  np.array([])
        self.last_expected =  np.array([])
        self.last_pose = Pose()
        self.last_scan_msg = LaserScan()
        self.window_size = window_size
        self.window_step = window_step
        self.threshold = threshold
        self.size_threshold = size_threshold
        self.range_max = 1
        self.covariance_threshold = covariance_threshold
        self.frame_id = frame_id
        self.pose_ready = False
        self.odom_topic = odom_topic
        self.moving = False
    def odom_callback(self, msg):
        l = msg.twist.twist.linear
        a = msg.twist.twist.angular
        self.moving = not np.allclose([l.x,l.y,l.z, a.x,a.y,a.z], np.zeros(6), atol=1e-2)
    def update_configuration(self,config, level):
        for key,value in config.items():
            if hasattr(self, key):
                rospy.loginfo('Setting {} to {}'.format(key, value))
                setattr(self, key, value)
        return config

    def window_stack(self, a):
        '''
        Function from here:
            https://stackoverflow.com/questions/15722324/sliding-window-in-numpy
        Returns a sliding window over a sample, a
        params
        ------
        a : np.array(m,n)
            Sample to slide over

        returns
        -------
        np.array(m,d)
            A stack of windowed samples, where d is the window size
        np.array(m,d)
            An array of the same size that gives the indicies of the windowed data
        '''
        wsz = self.window_size
        wsp = self.window_step
        num_windows = math.floor(float(len(a) - wsz + 1)/wsp)
        indexer = np.arange(wsz)[None, :] + wsp*np.arange(num_windows)[:, None]
        indexer = indexer.astype(np.int)
        return (a[indexer], indexer)
    def detect(self):
        """
        Detect objcets based on last scans

        """
        
        
        if not any(self.last_expected) or self.moving:
            self.out_scan_pub.publish(self.last_scan_msg)
            rospy.logwarn_throttle(5, "Robot is moving -- not detecting objects")
            return
        
	'''
        if  self.cov_mag > self.covariance_threshold:
            rospy.logwarn_throttle(5, "Covariance too high ({}) to filter reliably -- using raw scan".format(self.cov_mag))
            self.out_scan_pub.publish(self.last_scan_msg)
            return
	'''

        if self.last_scan.shape != self.last_expected.shape:
            rospy.logerr('Expected scan (n={}) is not the same size as received scan (n={})'.format(self.last_expected.shape,self.last_scan.shape))

        if np.array_equal(self.last_expected,self.last_scan):
            self.out_scan_pub.publish(self.last_scan_msg)
            return
        mr1 = self.last_scan_msg.range_max
        mr2 = self.last_expected_msg.range_max
        max_range = max([mr1, mr2])
        ls = self.last_scan/max_range
        ls[np.isinf(ls)] = 1
        els = self.last_expected/max_range
        els[np.isinf(els)] = 1
        # best_offset = self.get_best_offset_es(ls, els)
        best_offset= 0
        els = np.roll(els, best_offset)
        
        er2 = els - ls
        kernel = np.ones(self.window_size) * 1.0/self.window_size
        # Convolution allows for small gaps to be filled
        er2 = np.convolve(er2, kernel, mode='same')
        detected_objects = er2>self.threshold

        # Calculate center of mass of objects
        detected_objects_position = []
        in_object = False
        pos_begin, pos_end, i = 0, 0, 0
        while i < len(detected_objects):
            if detected_objects[i]:
                if not in_object:
                    pos_begin = i
                    in_object = True
            else:
                if in_object:
                    pos_end = i
                    # Offset accounts for the dilation from the convolution
                    offset = int(math.floor(self.window_size / 2))
                    detected_objects_position.append((pos_begin+offset, pos_end-offset))
                    in_object = False
            i += 1
        msg = NovelObjectArray()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = rospy.Time.now()
        for i,o in enumerate(detected_objects_position):
            pos_begin, pos_end = o
            com = int(math.floor((pos_begin+pos_end)/2))
            m = NovelObject()
            # s = self.calculate_segment_lengths(ls[pos_begin:pos_end])
            
            p = self.calculate_position_from_index(pos_begin, pos_end, ls*max_range)
            s = self.calculate_size(p, pos_end-pos_begin)
            if s >= self.size_threshold:
                m.pose = p
                m.size = s
                m.angular_size = pos_end - pos_begin
                # Make this covariance better?
                #m.pose.covariance = list(np.zeros((6,6)).flatten())
                m.id = i
                msg.detected_objects.append(m)
        self.detected_object_pub.publish(msg)

        # Publish filtered laser scan
        msg = deepcopy(self.last_scan_msg)
        u_range = np.array(msg.ranges)
        real_expected = els*max_range
        u_range[detected_objects] = real_expected[detected_objects]
        intensities = np.zeros(len(u_range))
        intensities[detected_objects] = 1.0
        msg.ranges = list(u_range)
        msg.intensities = intensities
        self.out_scan_pub.publish(msg)

        
    def get_best_offset_es(self, scan, expected_scan):
        best_offset = 0
        best_err = np.inf
        for i in range(len(scan)):
            es = np.roll(expected_scan, i)
            err = np.sum((scan-es)**2)
            if err < best_err:
                best_offset = i
                best_err = err
        return best_offset

    def calculate_position_from_index(self, begin, end, last_scan):

        median_index = int((begin+end)/2.0)
        rad = float(median_index)/len(last_scan) * 2 * math.pi
        # x = np.mean(last_scan[begin:end]) * math.cos(rad + math.pi) 
        # y = np.mean(last_scan[begin:end]) * math.sin(rad + math.pi)
        x = last_scan[median_index] * math.cos(rad) 
        y = last_scan[median_index] * math.sin(rad)
        return Pose(position=Point(x,y,0), orientation=Quaternion(w=1))
    def calculate_size(self, p, a_sz):
        p_ = p.position
        dist = p_.x**2 + p_.y**2
        dist = math.sqrt(dist)
        return 2*dist*math.tan(math.radians(a_sz)/2.0)
    def pose_callback(self, msg):
        self.last_pose = msg.pose
        #self.last_pose_covariance = np.array(msg.pose.covariance)
        #self.cov_mag = np.linalg.norm(self.last_pose_covariance)
        self.pose_ready = True
        # rospy.logwarn_throttle(10, 'Pose msg received: cov: {} mag: {}'.format(self.last_pose_covariance, self.cov_mag ))
        self.last_pose_header = msg.header
    def ls_callback(self, msg):
        self.last_scan_msg = msg
        if self.pose_ready:
            self.range_max = msg.range_max
            self.last_scan = np.array(msg.ranges)
            # self.pose_ready = False

    def els_callback(self, msg):
        self.last_expected = np.array(msg.ranges)
        self.last_expected_msg = msg
        self.detect()

    def calculate_segment_lengths(self, p):
        p1 = p[:-1]
        a1 = np.radians(range(len(p1)))
        p2 = p[1:]
        a2 = np.radians(range(1,len(p2)+1))
        p12 = p1 ** 2
        p22 = p2 ** 2
        o = p12 + p22 - 2*p1*p2*np.cos(a1-a2)
        return np.sum(np.sqrt(o))
