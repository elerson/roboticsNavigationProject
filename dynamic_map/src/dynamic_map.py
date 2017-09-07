#!/usr/bin/env python
import rospy
import json
import numpy as np
import math
import matplotlib.path as mplPath
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid
from robot_controller.msg import gps_coord
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
import tf

class DynamicMap:
    def __init__(self, input_map_info = '../config/experiment.json'):

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
        self.map = np.zeros((self.height, self.width, 3))


        self.gps_waypoints = self.mapdata['waypoints']
        self.gps_obstacles = self.mapdata['obstacles']
        self.waypoints = []
        self.obstacles = []
        self.pixel_waypoints = []
        self.pixel_obstacles = []
        self.obstacle_path = []

        self.current_gps_pos = []

        #open map info file


        #map service
        self.map_service = rospy.Service('get_map', getMap, self.getMapSrv)

        #publisher to map changes
        #TODO:
        

        #publisher to goal pose (map_coordinates)

        #subscriber to goal pose (GPS coordinates)


        #publisher to current pose (map_coordinates)
        self.current_pose_publisher = rospy.Publisher('current_position_map_coord', gps_coord, queue_size=1)
        #subscriber to gps curent pose (GPS coordinates)
        rospy.Subscriber("current_position_gps", gps_coord, self.convertGPSToMapCoord)

    def convertGPSToMapCoord(self, coord):

        self.current_gps_pos =  [coord.lat, coord.lng]
        pnt = self.fromLatLngToPoint(self.current_gps_pos[0], self.current_gps_pos[1], self.zoom)

        pixel_coord = self.fromPointToPixel(pnt[0], pnt[1])

        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'robot_position_map'
        pose.position.x = pixel_coord[0]
        pose.position.y = pixel_coord[1]
        pose.position.z = 0

        orientation = tf.transformations.quaternion_from_euler([0, 0, coord.orientation])
        pose.orientation = orientation
        self.current_pose_publisher.publish(pose)



    def getMapSrv(self):
        map_ = OccupancyGrid()
        map_.header.stamp = rospy.Time.now()
        map_.header.frame_id = 'map'

        map.info.resolution = 1
        map_.info.width = self.width
        map_.info.height = self.height

        map_.data.push(self.map)
        return getMapResponse(map_)

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
                        self.map[:,:,0] = 255
                        self.map[:,:,1] = 255
                        self.map[:,:,2] = 255



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
