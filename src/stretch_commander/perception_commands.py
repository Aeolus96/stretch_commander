
class StretchPerception:
    def __init__(self):
        #Initialize variables:
        self.all_raw_bbox_points =[]
        self.raw_bbox_points = []
        self.final_points =[]

        #holds dections from /yolo/results
        self.detections = []
        self.bbox_time = rospy(0)

        #PUBLISHERS/SUBSCRIBERS MAY NEED TO BE MODIFIED TO REFLECT THE CORRECT TOPICS
        self.bbox_sub = rospy.Subscriber('/yolo/results', Detection2DArray, self.bounding_box_callback)

        self.image_sub = rospy.Subscriber('/camera/depth/color/points', PointCloud2, point_cloud_callback)

        
        self.point_pub = rospy.Publisher("/CameraOutput", PointStamped, queue_size=5)
        self.point_pub.publish(self.final_points)


    def bounding_box_callback(self, boxes):
        #print(data)#MODIFIED TO REFLECT THE CORRECT TOPICS
        #self.bbox_sub = rospy.Subscriber('/Camera

        for detection in boxes:
            self.detections.append(detection)




    # Extract bounding box dimensions and convert
    def point_cloud_callback(self,pc_data):
        print("point cloud callback reached")
        
        all_filtered_points = []

         
       
        for detection in self.detections:

            #for testing:
            print(detection)


            # access the bounding box points
            bbox = detection.detections.bbox
            
            width = bbox.size_x
            height = bbox.size_y
            bbox_center_x = bbox.center.x
            bbox_center_y = bbox.center.y
            bbox_time = bbox.header.stamp

            '''just in case:
            top_left_corner = [bbox_center_x - width/2, bbox_center_y - height /2]
            top_right_corner = [bbox_center_x + width/2, bbox_center_y - height/2]
            bottom_left_corner = [bbox_center_x + width/2, bbox_center_y + height/2]
            bottom_right_corner =[bbox_center_x - width/2, bbox_center_y + height]
            '''

            xmin = bbox_center_x - width/2
            xmax = bbox_center_x + width/2
            ymin = bbox_center_y - height/2
            ymax = bbox_center_y + height/2

            #creates actual bounding box points, saves to global raw_bbox_points
            #raw_bbox_points = [xmin, ymin, xmax,  ymax]

            #to reduce any duplicates, will only add bbox to list (to be processed) if it doesn't already exist
            #if raw_bbox_points not in self.all_raw_bbox_points:
            #self.all_raw_bbox_points.append(raw_bbox_points)

            #for raw_bbox in self.all_raw_bbox_points

            point_to_grab=PointStamped()
            D3_bbox_points = []

            #(global) raw_bbox_points =[xmin, ymin, xmax,  ymax]
            # bbox globals
            #xMin = raw_bbox[0]
            #xMax = raw_bbox[2]
            #yMin = raw_bbox[1]
            #yMax = raw_bbox[3]

            # bbox pixels to D3 points
    
            #xyz_image=np.zeros((yMax-yMin,xMax-xMin,3),np.float32)
    
            row=0
            col=0
            for row in range(ymin,ymax):
                for col in range(xmin,xmax):
                    index = (row * pc_data.row_step) + (col * pc_data.point_step)
                    # Get the XYZ points [meters]
                    (X, Y, Z,rgb) = struct.unpack_from('fffl', pc_data.data, offset=index)

                    #create point stamped object to use when transformiing points:
                    D3_point =PointStamped()

                    #frame will eventually be 'usb_cam/image_raw'
                    D3_point.header.frame_id = 'camera_color_optical_frame'
                    D3_point.header.stamp = bbox_time

                    D3_point.point.x = X
                    D3_point.point.y = Y
                    D3_point.point.z = Z

                    # Append to array of D3 points in camera frame:
                    D3_bbox_points.append(D3_point)

                    #Transfrom D3 points to map frame
                    #transformation info:
                    curr_time = rospy.time(0)
                    tfBuffer = tf2_ros.Buffer()
                    listener = tf2_ros.TransformListener(tfBuffer);

                    try:
                        #from frame will be 'usb_cam/image_raw'
                        transform = tfBuffer.lookup_transform(
                            target_frame = 'map',
                            target_time = curr_time,
                            source_frame = 'camera_color_optical_frame',
                            source_time = bbox_time,
                            fixed_frame = 'base_link',
                            timeout=rospy.duration(1.0))

                        transformed_points = tfBuffer.transform(D3_bbox_points, 'base_link', rospy.Duration(1.0))
                        # Z height sorting and filtering clusters into a single point
                        point_to_grab = filter(transformed_points)

                        all_filtered_points.append(point_to_grab)
                        


                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                        print("error making transformation")
                        self.rate.sleep()
                        continue

        #These are the points that will be published
        self.final_points = self.cluster_points(self.all_converted_points)

        

    ######### FUNCTIONS USED IN THE BOUNDING BOX CALLBACK ##########
    def filter_points(self, points):
        #filters all D3 points within one bounding box, and returns the highest point

        #assuming floor height is 0.0 Confirm with team.
        #to use to filter out floor points
        floor_height=0.0

        #making max tshirt height 5 inches off ground. Confirm with team
        max_tshirt_height = 0.127

        #will return the highest point in the filtered array
        final_point=PointStamped(0,0,0)

        for point in points:
            #(if the point is not the floor and under 5 inches)
            if point.point.z>floor_height and point.point.z < max_tshirt_height:
                if point.point.z > final_point.point.z:
                    final_point = point
    
        return final_point


    def cluster_points(self, point_array):

        point_arr=point_array

        #THRESHOLD TO BE MODIFIED WITH ACTUAL DATA
        threshold = 5

        #go through all points, and combine any that are within the threshold of eachother:
        for i in range(len(point_arr)):
            #point to compare:
            current_point = point_arr[i]

            #array will hold all points that are close to eachother:
            points_to_merge=[]
            locations =[]
            #go through the rest of the points:
            for j in range(i+1, len(point_arr)):
                difference = self.find_distance(current_point, point_arr[j])
                if difference < threshold:
                    points_to_merge.append(point_arr[j])
                    locations.append(j)

            if len(points_to_merge) > 0:
                updated_point = self.find_average(current_point, points_to_merge)
                for index in locations:
                    point_arr[index]=updated_point

        #remove multiples:
        final_points=list(set(point_arr))

        #returns points 
        return final_points
            
 
    def find_distance(self, point1, point2):
        x1, y1, z1 = point1.point.x, point1.point.y, point1.point.z
        x2, y2, z2 = point2.point.x, point2.point.y, point2.point.z

        #differences:
        dx = x2 - x1
        dy = y2 - y1
        dz = z2 - z1

        #distance formula:
        distance = math.sqrt(dx**2 + dy**2 + dz**2)

        return distance



    def find_average(self, current_point, point_arr):
        #sums
        sum_x=current_point.point.x
        sum_y=current_point.point.y
        sum_z=current_point.point.z

        for point in point_arr:
            sum_x += point.point.x
            sum_y += point.point.y
            sum_z += point.point.z

        avg_x = sum_x /len(self.points)
        avg_y = sum_y /len(self.points)
        avg_z = sum_z /len(self.points)

        avg_point = PointStamped(point.header, self.Point(avg_x, avg_y, avg_z))

        return avg_point
