#!/usr/bin/env python
import rospy
import json
import numpy as np
import math
import matplotlib.path as mplPath
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid
from base_planner_cu.srv import GetPose
from robot_controller.msg import gps_coord
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose, Point
from nav_msgs.msg import Path
import tf
import std_msgs.msg


class DynamicMap:
    def __init__(self, input_map_info = '../config/experiments.json'):

        #open experiment data
        with open(input_map_info) as data_file:    
            self.mapdata = json.load(data_file)

        #global variables
        self.zoom = self.mapdata['zoom']
        self.width = self.mapdata['width']
        self.height = self.mapdata['height']

        self.x_top_left = self.mapdata['x_top_left']
        self.y_top_left = self.mapdata['y_top_left']

        self.numtiles_x = self.mapdata['numtiles_x']
        self.numtiles_y = self.mapdata['numtiles_y']

        #create map
        self.map = np.zeros((self.height, self.width))


        self.gps_waypoints = self.mapdata['waypoints']
        self.gps_obstacles = self.mapdata['obstacles']
        self.waypoints = []
        self.obstacles = []
        self.pixel_waypoints = []
        self.pixel_obstacles = []
        self.obstacle_path = []

        self.current_gps_pos = []

        ##calculate the latitude for the central posi

        x = self.x_top_left + self.numtiles_x/2
        y = self.y_top_left + self.numtiles_x/2
        latlng = self.fromPointToLatLng(x, y, self.zoom)
        self.ground_resolution = (math.cos(latlng[0] * math.pi/180) * 2 * math.pi * 6378137) / (256 * math.pow(2, self.zoom))


        ##
        ##  MAP
        ##
        #map service
        self.map_service = rospy.Service('get_map_cu', GetMap, self.getMapSrv)

        #pose service
        self.pose_service = rospy.Service('get_pose_cu', GetPose, self.getPoseSrv)

        #publisher to map changes
        
        ##
        ##  Navigation coordinates
        ##
        # Subscribe to naviation path in pixel coordinates
        rospy.Subscriber("global_path_cu", Path, self.convertPathToGPS)
        
        # Publish to naviation path in GPS coordinates
        self.navigation_path_publisher = rospy.Publisher('global_path_gps', Path, queue_size=1)
       
        

        ##
        ##  Current Goal
        ##
        #publisher to goal pose (map_coordinates)
        #TODO:
        self.goal_publisher = rospy.Publisher('goal_cu', PoseStamped, queue_size=1)
        #subscriber to goal pose (GPS coordinates)
        #TODO
        rospy.Subscriber("goal_gps", gps_coord, self.convertGoalFromGPSToMap)

        self.pub = rospy.Publisher('topic_name', std_msgs.msg.String, queue_size=10)
        
        ##
        ##   Current pose 
        ##
        #publisher to current pose (map_coordinates)
        self.current_pose_publisher = rospy.Publisher('current_position_map_coord', PoseStamped, queue_size=100)
        #subscriber to gps curent pose (GPS coordinates)
        rospy.Subscriber("current_position_gps", gps_coord, self.convertCurrentPosFromGPSToMap)


        self.createMap()
        print("initialized")


   
    def convertPathToGPS(self, path):
        path.header.frame_id = 'nav_path_gps'

        for i in range(len(path.poses)):
            pixel_pos_x = path.poses[i].pose.position.x/self.ground_resolution
            pixel_pos_y = path.poses[i].pose.position.y/self.ground_resolution

            point = self.fromPixelToPoint(pixel_pos_x, pixel_pos_y)
            gps =  self.fromPointToLatLng(point[0], point[1], self.zoom)

            #Put the lat in x and lng in y //Be carreful
            path.poses[i].pose.position.x = gps[0]
            path.poses[i].pose.position.y = gps[1]


        self.navigation_path_publisher.publish(path)



    def convertGoalFromGPSToMap(self, coord):

        self.current_gps_pos =  [coord.lat, coord.lng]
        pnt = self.fromLatLngToPoint(self.current_gps_pos[0], self.current_gps_pos[1], self.zoom)

        pixel_coord = self.fromPointToPixel(pnt[0], pnt[1])

        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'robot_position_map_coord'
        pose.pose.position.x = pixel_coord[0]*self.ground_resolution
        pose.pose.position.y = pixel_coord[1]*self.ground_resolution
        pose.pose.position.z = 0

        orientation = tf.transformations.quaternion_from_euler(0, 0, coord.orientation)
        pose.pose.orientation.x = orientation[0]
        pose.pose.orientation.y = orientation[1]
        pose.pose.orientation.z = orientation[2]
        pose.pose.orientation.w = orientation[3]

        self.goal_publisher.publish(pose)

    

    def convertCurrentPosFromGPSToMap(self, coord):

        self.current_gps_pos =  [coord.lat, coord.lng]
        pnt = self.fromLatLngToPoint(self.current_gps_pos[0], self.current_gps_pos[1], self.zoom)

        pixel_coord = self.fromPointToPixel(pnt[0], pnt[1])

        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'robot_position_map_coord'
        pose.pose.position.x = pixel_coord[0]*self.ground_resolution
        pose.pose.position.y = pixel_coord[1]*self.ground_resolution
        pose.pose.position.z = 0

        orientation = tf.transformations.quaternion_from_euler(0, 0, coord.orientation)
        pose.pose.orientation.x = orientation[0]
        pose.pose.orientation.y = orientation[1]
        pose.pose.orientation.z = orientation[2]
        pose.pose.orientation.w = orientation[3]
        
        
        self.curr_pose = pose; #save current pose
        self.current_pose_publisher.publish(pose)



    def getPoseSrv(self):
        return GetPoseResponse(self.curr_pose)

    def getMapSrv(self):
        map_ = OccupancyGrid()
        map_.header.stamp = rospy.Time.now()
        map_.header.frame_id = 'map'

        map.info.resolution = self.ground_resolution
        map_.info.width = self.width
        map_.info.height = self.height

        map_.info.origin = Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1))

        flat_map = self.map.reshape((self.self.map.size,))
        map_.data = list(np.round(flat_map))

        return GetMapResponse(map_)
    

    def createMap(self):
        self.loadObstacles()

        for obstacle in self.pixel_obstacles:
            print('defining obstacles ... wait')
            #get min and max (x,y)
            min_x = float('inf')
            max_x = float('-inf')
            min_y = float('inf')
            max_y = float('-inf')
            for pt in obstacle:
                min_x = min(pt[0], min_x)
                max_x = max(pt[0], max_x)
                min_y = min(pt[1], min_y)
                max_y = max(pt[1], max_y)


            for x in range(int(min_x), int(max_x)):
                for y in range(int(min_y), int(max_y)):          
                    if(self.isPixelInsideObstacle(x, y)):
                        self.map[x,y] = 255
            print('defining obstacles ... done')



    def isPixelInsideObstacle(self, x, y):
        for i in range(len(self.obstacle_path)):
            if self.obstacle_path[i].contains_point((x, y)) :
                return True
        return False


    def loadWaypoints(self):
        for wp in self.gps_waypoints:   
            pt = self.fromLatLngToPoint(wp[0], wp[1], self.zoom)
            self.waypoints.append(pt)
            self.pixel_waypoints.append(self.fromPointToPixel(pt[0], pt[1]))


    def loadObstacles(self):
        for obstacle in self.gps_obstacles:
            obstacle_ = []
            pixel_obstacles_ = []
            for gps_pt in obstacle:
                pt = self.fromLatLngToPoint(gps_pt[0], gps_pt[1], self.zoom)
                obstacle_.append(pt)
                pixel_obstacles_.append(self.fromPointToPixel(pt[0], pt[1]))
            self.obstacles.append(obstacle_)
            self.pixel_obstacles.append(pixel_obstacles_)
            self.obstacle_path.append(mplPath.Path(pixel_obstacles_))
        

    def fromPixelToPoint(self, x, y):
        return [(x/self.width)*self.numtiles_x + self.x_top_left, (y/self.height)*self.numtiles_y + self.y_top_left]


    def fromPointToPixel(self, x, y):
        return [((x-self.x_top_left)/self.numtiles_x)*self.width, ((y-self.y_top_left)/self.numtiles_y)*self.height]


    def fromPointToLatLng(self, x, y, zoom):

        x_ = (x/(math.pow(2, zoom)))*256;
        y_ = (y/(math.pow(2, zoom)))*256;

        lng = ((x_ / 256) * 360) - 180;
        n = math.pi - ((2 * math.pi * y_) / 256);
        lat = ((180 / math.pi) * math.atan(0.5 * (math.exp(n) - math.exp(-n))));

        return [lat, lng]


    def fromLatLngToPoint(self, lat, lng, zoom):
        x_ = ((lng + 180) / 360) * 256;
        y_ = ((1 - math.log(math.tan(lat * math.pi / 180) + 1 / math.cos(lat * math.pi / 180)) / math.pi) / 2 * math.pow(2, 0)) * 256

        x = x_*(math.pow(2, zoom)/256)
        y = y_*(math.pow(2, zoom)/256)

        return  [x, y]

    def run(self):
        ## Run Dynamic map algorithm



        #sleep for while
        rospy.sleep(0.1)



def init_current_node():
    rospy.init_node('dynamic_map_', anonymous=True)
    map_param = rospy.get_param('~map_info', '../config/experiment.json')


    dm = DynamicMap(map_param)
    while not rospy.is_shutdown():
        #print('running...')
        dm.run()

if __name__ == '__main__':
    init_current_node()
