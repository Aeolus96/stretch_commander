import rospy
from std_msgs.msg import Bool
import tf2_ros
import tf2_geometry_msgs
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from vision_msgs.msg import Detection2D, Detection2DArray, BoundingBox2D, BoundingBox2DArray
import cv2
import numpy as np
import pdb
import struct
import ctypes
import math


class StretchPerception:
    def __init__(self):
        # Subscribers
        self.bbox_topic = "/yolo/results"
        self.camera_image_topic = "/camera/depth/color/points"
        self.bbox_sub = rospy.Subscriber(self.bbox_topic, Detection2DArray, self.bounding_box_callback)
        self.image_sub = rospy.Subscriber(self.camera_image_topic, PointCloud2, self.point_cloud_callback)
        # self.stretch_sub=rospy.Subscriber("/tf/map", tf, self.stretch_location_callback)

        # Publishers
        self.trigger_scan_topic = "/trigger_yolo/"
        self.target_point_topic = "/target_point"
        self.point_pub = rospy.Publisher(self.target_point_topic, PointStamped, queue_size=5)
        self.trigger_yolo_pub = rospy.Publisher(self.trigger_scan_topic, Bool, queue_size=5)

        # Initialize variables and buffers:
        self.all_raw_bbox_points = []
        self.raw_bbox_points = []
        self.final_points = []
        self.detections = []  # holds detections from /yolo/results
        self.bbox_time = rospy.Time()

        self.detected_objects = False

    def trigger_yolo(self):
        msg = Bool()
        msg.data = True
        self.trigger_yolo_pub.publish(msg)

    ################# BOUNDING BOX CALLBACK FUNCTIONS##########################

    def bounding_box_callback(self, boxes):
        # print(data)#MODIFIED TO REFLECT THE CORRECT TOPICS
        # self.bbox_sub = rospy.Subscriber('/Camera

        for detection in boxes.detections:
            self.detections.append(detection)
            self.detected_objects = True

    ################# POINT CLOUD CALLBACK FUNCTIONS###########################

    # Extract bounding box dimensions and convert
    def point_cloud_callback(self, pc_data):
        # print("point cloud callback reached")

        all_filtered_points = []

        for detection in self.detections:
            print("detection: ", detection)
            # for testing:
            # print(detection)

            # access the bounding box points
            bbox = detection.bbox

            width = bbox.size_x
            height = bbox.size_y
            bbox_center_x = bbox.center.x
            bbox_center_y = bbox.center.y
            bbox_time = detection.header.stamp

            xmin = bbox_center_x - width / 2
            xmax = bbox_center_x + width / 2
            ymin = bbox_center_y - height / 2
            ymax = bbox_center_y + height / 2

            # creates actual bounding box points, saves to global raw_bbox_points
            # raw_bbox_points = [xmin, ymin, xmax,  ymax]

            # to reduce any duplicates, will only add bbox to list (to be processed) if it doesn't already exist
            # if raw_bbox_points not in self.all_raw_bbox_points:
            # self.all_raw_bbox_points.append(raw_bbox_points)

            # for raw_bbox in self.all_raw_bbox_points

            point_to_grab = PointStamped()
            D3_bbox_points = []

            # (global) raw_bbox_points =[xmin, ymin, xmax,  ymax]
            # bbox globals
            # xMin = raw_bbox[0]
            # xMax = raw_bbox[2]
            # yMin = raw_bbox[1]
            # yMax = raw_bbox[3]

            # bbox pixels to D3 points

            # xyz_image=np.zeros((yMax-yMin,xMax-xMin,3),np.float32)

            for row in range(int(ymin), int(ymax)):
                for col in range(int(xmin), int(xmax)):
                    index = (row * pc_data.row_step) + (col * pc_data.point_step)
                    print("Index: ", index)
                    # Get the XYZ points [meters]
                    try:
                        (X, Y, Z) = struct.unpack_from("ffff", pc_data.data, offset=index)
                        
                        
                    except struct.error:
                        print("Error unpacking point at index: ", index)
                        continue

                    print("3D X point: ", X)
                    print("3D Y point: ", Y)
                    print("3D Z point: ", Z)
                    # create point stamped object to use when transforming points:
                    D3_point = PointStamped()

                    # frame will eventually be 'usb_cam/image_raw'
                    D3_point.header.frame_id = "camera_color_optical_frame"
                    D3_point.header.stamp = bbox_time

                    D3_point.point.x = X
                    D3_point.point.y = Y
                    D3_point.point.z = Z

                    # Append to array of D3 points in camera frame:
                    D3_bbox_points.append(D3_point)

            # Transfrom D3 points to map frame
            # transformation info:
            curr_time = rospy.Time(0)
            tfBuffer = tf2_ros.Buffer()
            listener = tf2_ros.TransformListener(tfBuffer)
            try:
                # from frame will be 'usb_cam/image_raw'
                transform = tfBuffer.lookup_transform_full(
                    target_frame="map",
                    target_time=curr_time,
                    source_frame="camera_color_optical_frame",
                    source_time=bbox_time,
                    fixed_frame="base_link",
                    timeout=rospy.Duration(1.0),
                )

                transformed_points = [
                    tf2_geometry_msgs.do_transform_point(point, transform) for point in D3_bbox_points
                ]
                # Z height sorting and filtering clusters into a single point
                point_to_grab = self.filter_points(transformed_points)
                print("Point to grab: ", point_to_grab)
                all_filtered_points.append(point_to_grab)

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as error:
                print("error making transformation: ", error)

        if all_filtered_points:
            print("all filtered points: ", all_filtered_points)
            # These are the points that will be published
            self.final_points = self.cluster_points(all_filtered_points)
            print(self.final_points)
            self.point_pub(self.final_points, PointStamped)
            self.final_points = []

    def filter_points(self, points):
        # filters all D3 points within one bounding box, and returns the highest point

        # assuming floor height is 0.0 Confirm with team.
        # to use to filter out floor points
        floor_height = 0.0

        # making max tshirt height 5 inches off ground. Confirm with team
        max_tshirt_height = 0.127

        # will return the highest point in the filtered array
        final_point = PointStamped()
        # header must be changed to correct header
        final_point.header.frame_id = "camera_color_optical_frame"
        final_point.point.x = 0.0
        final_point.point.y = 0.0
        final_point.point.z = 0.0

        for point in points:
            # (if the point is not the floor and under 5 inches)
            if point.point.z > floor_height and point.point.z < max_tshirt_height:
                if point.point.z > final_point.point.z:
                    final_point = point

        return final_point

    def cluster_points(self, point_array):
        print("Array passed into cluster_points():", point_array)
        point_arr = point_array
        final_point = PointStamped()

        final_point.header = "map"
        final_point.point.x = 0.0
        final_point.point.y = 0.0
        final_point.point.z = 0.0

        # THRESHOLD TO BE MODIFIED WITH ACTUAL DATA
        threshold = 5

        # go through all points, and combine any that are within the threshold of eachother:

        for i in range(len(point_arr)):
            # point to compare:
            current_point = point_arr[i]
            # array will hold all points that are close to eachother:
            points_to_merge = []
            locations = []
            # go through the rest of the points:
            for j in range(i + 1, len(point_arr)):
                difference = self.find_distance(current_point, point_arr[j])
                if difference < threshold:
                    points_to_merge.append(point_arr[j])
                    locations.append(j)

            if len(points_to_merge) > 0:
                updated_point = self.find_average(current_point, points_to_merge)
                for index in locations:
                    point_arr[index].pop(index)
                point_arr.append(updated_point)

        print("point_arr: ", point_arr)

        final_point = self.find_average(point_arr[0], point_arr)
        self.final_points = final_point

    def find_distance(self, point1, point2):
        x1, y1, z1 = point1.point.x, point1.point.y, point1.point.z
        x2, y2, z2 = point2.point.x, point2.point.y, point2.point.z

        # differences:
        dx = x2 - x1
        dy = y2 - y1
        dz = z2 - z1

        # distance formula:
        distance = math.sqrt(dx**2 + dy**2 + dz**2)

        return distance

    def find_average(self, current_point, point_arr):
        print("From inside 'find_average': ")
        print("current/first point length: ", len(current_point))
        print("length of rest of array: ", len(point_arr))

        print("current/first point: ", current_point)
        print("rest of array: ", point_arr)

        # sums
        sum_x = current_point.point.x
        sum_y = current_point.point.y
        sum_z = current_point.point.z

        for point in point_arr:
            sum_x += point.point.x
            sum_y += point.point.y
            sum_z += point.point.z

        avg_x = sum_x / len(self.points)
        avg_y = sum_y / len(self.points)
        avg_z = sum_z / len(self.points)

        avg_point = PointStamped(point.header, self.Point(avg_x, avg_y, avg_z))

        return avg_point

    ######### FUNCTIONS USED IN STRETCH LOCATION CALLBACK ##########

    # takes in all final points and returns a list of points in ascending order based on their distance from the robot
    """
    def stretch_location_callback(self, stretch_location):
        distances = []
        
        stretch_x = stretch_location. #What topic/type?
        stretch_Y = stretch_location. #what topic/type?
        stretch_Z = stretch_location. #what topic/type?
        
        for detection in self.final_points:
            detected_X = detection.point.x
            detected_Y = detection.point.y
            detected_Z = detection.point.z
            
            distance=math.sqrt((detected_X-stretch_x)**2+(detected_Y-stretch_Y)**2+(detected_Z-stretch_Z)**2)
            distances.append(distance)
            
        length=len(distances)
        if length>=1:
            self.point_pub(self.final_points[0])
        else:
            for i in range(length):
                for j in range(length-i-1):
                    if distances[j]>distances[j+1]:
                        distances[j],distances[j+1]=distances[j+1],distances[j]
                        self.final_points[j],self.final_points[j+1]=self.final_points[j+1],self.final_points[j]
        
            for points in self.final_points:
                self.point_pub(points)
            
            
        
        
        
        
        
        for point in self.final_points:
            distances.append(self.find_distance(point, robot))
    """
