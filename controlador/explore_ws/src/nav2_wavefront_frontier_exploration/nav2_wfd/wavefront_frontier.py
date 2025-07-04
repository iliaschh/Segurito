import sys
import time
from nav2_msgs.srv import SaveMap
from std_srvs.srv import SetBool
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
# from nav2_msgs.action import FollowWaypoints
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ManageLifecycleNodes
from nav2_msgs.srv import GetCostmap
from nav2_msgs.msg import Costmap
from nav_msgs.msg  import OccupancyGrid
from nav_msgs.msg import Odometry

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile
import os
from enum import Enum

import numpy as np

import math

OCC_THRESHOLD = 10
MIN_FRONTIER_SIZE = 5

class Costmap2d():
    class CostValues(Enum):
        FreeSpace = 0
        InscribedInflated = 253
        LethalObstacle = 254
        NoInformation = 255
    
    def __init__(self, map):
        self.map = map

    def getCost(self, mx, my):
        return self.map.data[self.__getIndex(mx, my)]

    def getSize(self):
        return (self.map.metadata.size_x, self.map.metadata.size_y)

    def getSizeX(self):
        return self.map.metadata.size_x

    def getSizeY(self):
        return self.map.metadata.size_y

    def __getIndex(self, mx, my):
        return my * self.map.metadata.size_x + mx

class OccupancyGrid2d():
    class CostValues(Enum):
        FreeSpace = 0
        InscribedInflated = 100
        LethalObstacle = 100
        NoInformation = -1

    def __init__(self, map):
        self.map = map

    def getCost(self, mx, my):
        return self.map.data[self.__getIndex(mx, my)]

    def getSize(self):
        return (self.map.info.width, self.map.info.height)

    def getSizeX(self):
        return self.map.info.width

    def getSizeY(self):
        return self.map.info.height

    def mapToWorld(self, mx, my):
        wx = self.map.info.origin.position.x + (mx + 0.5) * self.map.info.resolution
        wy = self.map.info.origin.position.y + (my + 0.5) * self.map.info.resolution

        return (wx, wy)

    def worldToMap(self, wx, wy):
        if (wx < self.map.info.origin.position.x or wy < self.map.info.origin.position.y):
            raise Exception("World coordinates out of bounds")

        mx = int((wx - self.map.info.origin.position.x) / self.map.info.resolution)
        my = int((wy - self.map.info.origin.position.y) / self.map.info.resolution)
        
        if  (my > self.map.info.height or mx > self.map.info.width):
            raise Exception("Out of bounds")

        return (mx, my)

    def __getIndex(self, mx, my):
        return my * self.map.info.width + mx

class FrontierCache():
    cache = {}

    def getPoint(self, x, y):
        idx = self.__cantorHash(x, y)

        if idx in self.cache:
            return self.cache[idx]

        self.cache[idx] = FrontierPoint(x, y)
        return self.cache[idx]

    def __cantorHash(self, x, y):
        return (((x + y) * (x + y + 1)) / 2) + y

    def clear(self):
        self.cache = {}

class FrontierPoint():
    def __init__(self, x, y):
        self.classification = 0
        self.mapX = x
        self.mapY = y

def centroid(arr):
    arr = np.array(arr)
    length = arr.shape[0]
    sum_x = np.sum(arr[:, 0])
    sum_y = np.sum(arr[:, 1])
    return sum_x/length, sum_y/length

def findFree(mx, my, costmap):
    fCache = FrontierCache()

    bfs = [fCache.getPoint(mx, my)]

    while len(bfs) > 0:
        loc = bfs.pop(0)

        if costmap.getCost(loc.mapX, loc.mapY) == OccupancyGrid2d.CostValues.FreeSpace.value:
            return (loc.mapX, loc.mapY)

        for n in getNeighbors(loc, costmap, fCache):
            if n.classification & PointClassification.MapClosed.value == 0:
                n.classification = n.classification | PointClassification.MapClosed.value
                bfs.append(n)

    return (mx, my)

def getFrontier(pose, costmap, logger):
    fCache = FrontierCache()

    fCache.clear()

    mx, my = costmap.worldToMap(pose.position.x, pose.position.y)

    freePoint = findFree(mx, my, costmap)
    start = fCache.getPoint(freePoint[0], freePoint[1])
    start.classification = PointClassification.MapOpen.value
    mapPointQueue = [start]

    frontiers = []

    while len(mapPointQueue) > 0:
        p = mapPointQueue.pop(0)

        if p.classification & PointClassification.MapClosed.value != 0:
            continue

        if isFrontierPoint(p, costmap, fCache):
            p.classification = p.classification | PointClassification.FrontierOpen.value
            frontierQueue = [p]
            newFrontier = []

            while len(frontierQueue) > 0:
                q = frontierQueue.pop(0)

                if q.classification & (PointClassification.MapClosed.value | PointClassification.FrontierClosed.value) != 0:
                    continue

                if isFrontierPoint(q, costmap, fCache):
                    newFrontier.append(q)

                    for w in getNeighbors(q, costmap, fCache):
                        if w.classification & (PointClassification.FrontierOpen.value | PointClassification.FrontierClosed.value | PointClassification.MapClosed.value) == 0:
                            w.classification = w.classification | PointClassification.FrontierOpen.value
                            frontierQueue.append(w)

                q.classification = q.classification | PointClassification.FrontierClosed.value

            
            newFrontierCords = []
            for x in newFrontier:
                x.classification = x.classification | PointClassification.MapClosed.value
                newFrontierCords.append(costmap.mapToWorld(x.mapX, x.mapY))

            if len(newFrontier) > MIN_FRONTIER_SIZE:
                frontiers.append(centroid(newFrontierCords))

        for v in getNeighbors(p, costmap, fCache):
            if v.classification & (PointClassification.MapOpen.value | PointClassification.MapClosed.value) == 0:
                if any(costmap.getCost(x.mapX, x.mapY) == OccupancyGrid2d.CostValues.FreeSpace.value for x in getNeighbors(v, costmap, fCache)):
                    v.classification = v.classification | PointClassification.MapOpen.value
                    mapPointQueue.append(v)

        p.classification = p.classification | PointClassification.MapClosed.value

    return frontiers
        

def getNeighbors(point, costmap, fCache):
    neighbors = []

    for x in range(point.mapX - 1, point.mapX + 2):
        for y in range(point.mapY - 1, point.mapY + 2):
            if (x > 0 and x < costmap.getSizeX() and y > 0 and y < costmap.getSizeY()):
                neighbors.append(fCache.getPoint(x, y))

    return neighbors

def isFrontierPoint(point, costmap, fCache):
    if costmap.getCost(point.mapX, point.mapY) != OccupancyGrid2d.CostValues.NoInformation.value:
        return False

    hasFree = False
    for n in getNeighbors(point, costmap, fCache):
        cost = costmap.getCost(n.mapX, n.mapY)

        if cost > OCC_THRESHOLD:
            return False

        if cost == OccupancyGrid2d.CostValues.FreeSpace.value:
            hasFree = True

    return hasFree

class PointClassification(Enum):
    MapOpen = 1
    MapClosed = 2
    FrontierOpen = 4
    FrontierClosed = 8

class WaypointFollowerTest(Node):

    def __init__(self):
        super().__init__(node_name='nav2_waypoint_tester', namespace='')
        self.waypoints = None
        self.readyToMove = True
        self.currentPose = None
        self.lastWaypoint = None
        # self.action_client = ActionClient(self, FollowWaypoints, 'FollowWaypoints')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped,
                                                      'initialpose', 10)

        self.costmapClient = self.create_client(GetCostmap, '/global_costmap/get_costmap')
        while not self.costmapClient.wait_for_service(timeout_sec=1.0):
            self.info_msg('service not available, waiting again...')
        self.initial_pose_received = False
        self.goal_handle = None

        pose_qos = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          # reliability=QoSReliabilityPolicy.BEST_EFFORT,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=1)

        self.model_pose_sub = self.create_subscription(Odometry,
          '/odom', self.poseCallback, 10) # pose_qos)

        # self.costmapSub = self.create_subscription(Costmap(), '/global_costmap/costmap_raw', self.costmapCallback, pose_qos)
        self.costmapSub = self.create_subscription(OccupancyGrid(), '/map', self.occupancyGridCallback, pose_qos)
        self.costmap = None

        self.get_logger().info('Running Waypoint Test')

    def occupancyGridCallback(self, msg):
        self.costmap = OccupancyGrid2d(msg)
    # método auxiliar para pausar el nav cuando se guarda el mapa
    def pause_nav(self, value: bool = True):
        cli = self.create_client(SetBool, '/controller_server/set_paused')
        if not cli.service_is_ready():
            self.warn_msg('set_paused no disponible')
            return

        req = SetBool.Request(data=value)
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if future.result() and future.result().success:
            state = "PAUSADO" if value else "REANUDADO"
            self.info_msg(f'Nav2 {state}')
        else:
            self.warn_msg('No se pudo pausar Nav2')

    def save_map(self):
        client = self.create_client(SaveMap, '/map_saver/save_map')
        if not client.service_is_ready():
            self.warn_msg("map_saver service no disponible")
            return

        home = os.path.expanduser("~")
        folder = os.path.join(home, "maps")
        os.makedirs(folder, exist_ok=True)
        filename = time.strftime("map_%Y%m%d_%H%M%S")
        req = SaveMap.Request()
        req.map_topic   = 'map'
        req.map_url     = os.path.join(folder, filename)
        req.image_format = 'png'

        self.info_msg(f"Guardando mapa en {req.map_url}.yaml …")
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() and future.result().success:
            self.info_msg("Mapa guardado con éxito")
            self.pause_nav(True)
        else:
            self.error_msg("Error al guardar mapa")
            self.pause_nav(True)

    def moveToFrontiers(self):
        try:
            frontiers = getFrontier(self.currentPose, self.costmap, self.get_logger())
        except Exception as e:
            self.warn_msg(f'Frontier search skipped: {e}')
            rclpy.timer.sleep(1.0)            # o spin_once(...)
            self.moveToFrontiers()
            return

        if len(frontiers) == 0:
            self.info_msg('No More Frontiers')
            self.save_map() 
            return

        location = None
        largestDist = 0
        for f in frontiers:
            dist = math.sqrt(((f[0] - self.currentPose.position.x)**2) + ((f[1] - self.currentPose.position.y)**2))
            if  dist > largestDist:
                largestDist = dist
                location = [f] 

        #worldFrontiers = [self.costmap.mapToWorld(f[0], f[1]) for f in frontiers]
        self.info_msg(f'World points {location}')
        self.setWaypoints(location)

        # action_request = FollowWaypoints.Goal()
        action_request = NavigateToPose.Goal()
        # action_request.poses = self.waypoints
        action_request.pose = self.waypoints[0]

        self.info_msg('Sending goal request...')
        send_goal_future = self.action_client.send_goal_async(action_request)
        try:
            rclpy.spin_until_future_complete(self, send_goal_future)
            self.goal_handle = send_goal_future.result()
        except Exception as e:
            self.error_msg('Service call failed %r' % (e,))

        if not self.goal_handle.accepted:
            self.error_msg('Goal rejected')
            return

        self.info_msg('Goal accepted')

        get_result_future = self.goal_handle.get_result_async()

        # self.info_msg("Waiting for 'FollowWaypoints' action to complete")
        self.info_msg("Waiting for 'NavigateToPose' action to complete")
        try:
            rclpy.spin_until_future_complete(self, get_result_future)
            status = get_result_future.result().status
            result = get_result_future.result().result
        except Exception as e:
            self.error_msg('Service call failed %r' % (e,))

        #self.currentPose = self.waypoints[len(self.waypoints) - 1].pose

        self.moveToFrontiers()

    def costmapCallback(self, msg):
        self.costmap = Costmap2d(msg)

        unknowns = 0
        for x in range(0, self.costmap.getSizeX()):
            for y in range(0, self.costmap.getSizeY()):
                if self.costmap.getCost(x, y) == 255:
                    unknowns = unknowns + 1
        self.get_logger().info(f'Unknowns {unknowns}')
        self.get_logger().info(f'Got Costmap {len(getFrontier(None, self.costmap, self.get_logger()))}')

    def dumpCostmap(self):
        costmapReq = GetCostmap.Request()
        self.get_logger().info('Requesting Costmap')
        costmap = self.costmapClient.call(costmapReq)
        self.get_logger().info(f'costmap resolution {costmap.specs.resolution}')

    def setInitialPose(self, pose):
        self.init_pose = PoseWithCovarianceStamped()
        self.init_pose.pose.pose.position.x = pose[0]
        self.init_pose.pose.pose.position.y = pose[1]
        self.init_pose.header.frame_id = 'map'
        self.currentPose = self.init_pose.pose.pose
        self.publishInitialPose()
        time.sleep(5)

    def poseCallback(self, msg):
        if (not self.initial_pose_received):
          self.info_msg('Received amcl_pose')
        self.currentPose = msg.pose.pose
        self.initial_pose_received = True


    def setWaypoints(self, waypoints):
        self.waypoints = []
        for wp in waypoints:
            msg = PoseStamped()
            msg.header.frame_id = 'map'
            msg.pose.position.x = wp[0]
            msg.pose.position.y = wp[1]
            msg.pose.orientation.w = 1.0
            self.waypoints.append(msg)

    def run(self, block):
        if not self.waypoints:
            rclpy.error_msg('Did not set valid waypoints before running test!')
            return False

        while not self.action_client.wait_for_server(timeout_sec=1.0):
            # self.info_msg("'FollowWaypoints' action server not available, waiting...")
            self.info_msg("'NavigateToPose' action server not available, waiting...")

        # action_request = FollowWaypoints.Goal()
        action_request = NavigateToPose.Goal()
        # action_request.poses = self.waypoints
        action_request.pose = self.waypoints[0]

        self.info_msg('Sending goal request...')
        send_goal_future = self.action_client.send_goal_async(action_request)
        try:
            rclpy.spin_until_future_complete(self, send_goal_future)
            self.goal_handle = send_goal_future.result()
        except Exception as e:
            self.error_msg('Service call failed %r' % (e,))

        if not self.goal_handle.accepted:
            self.error_msg('Goal rejected')
            return False

        self.info_msg('Goal accepted')
        if not block:
            return True

        get_result_future = self.goal_handle.get_result_async()

        # self.info_msg("Waiting for 'FollowWaypoints' action to complete")
        self.info_msg("Waiting for 'NavigateToPose' action to complete")
        try:
            rclpy.spin_until_future_complete(self, get_result_future)
            status = get_result_future.result().status
            result = get_result_future.result().result
        except Exception as e:
            self.error_msg('Service call failed %r' % (e,))

        if status != GoalStatus.STATUS_SUCCEEDED:
            self.info_msg('Goal failed with status code: {0}'.format(status))
            return False
        if len(result.missed_waypoints) > 0:
            self.info_msg('Goal failed to process all waypoints,'
                          ' missed {0} wps.'.format(len(result.missed_waypoints)))
            return False

        self.info_msg('Goal succeeded!')
        return True

    def publishInitialPose(self):
        self.initial_pose_pub.publish(self.init_pose)

    def shutdown(self):
        self.info_msg('Shutting down')

        self.action_client.destroy()
        # self.info_msg('Destroyed FollowWaypoints action client')
        self.info_msg('Destroyed NavigateToPose action client')

        transition_service = 'lifecycle_manager_navigation/manage_nodes'
        mgr_client = self.create_client(ManageLifecycleNodes, transition_service)
        while not mgr_client.wait_for_service(timeout_sec=1.0):
            self.info_msg(transition_service + ' service not available, waiting...')

        req = ManageLifecycleNodes.Request()
        req.command = ManageLifecycleNodes.Request().SHUTDOWN
        future = mgr_client.call_async(req)
        try:
            rclpy.spin_until_future_complete(self, future)
            future.result()
        except Exception as e:
            self.error_msg('%s service call failed %r' % (transition_service, e,))

        self.info_msg('{} finished'.format(transition_service))

        transition_service = 'lifecycle_manager_localization/manage_nodes'
        mgr_client = self.create_client(ManageLifecycleNodes, transition_service)
        while not mgr_client.wait_for_service(timeout_sec=1.0):
            self.info_msg(transition_service + ' service not available, waiting...')

        req = ManageLifecycleNodes.Request()
        req.command = ManageLifecycleNodes.Request().SHUTDOWN
        future = mgr_client.call_async(req)
        try:
            rclpy.spin_until_future_complete(self, future)
            future.result()
        except Exception as e:
            self.error_msg('%s service call failed %r' % (transition_service, e,))

        self.info_msg('{} finished'.format(transition_service))

    def cancel_goal(self):
        cancel_future = self.goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel_future)

    def info_msg(self, msg: str):
        self.get_logger().info(msg)

    def warn_msg(self, msg: str):
        self.get_logger().warn(msg)

    def error_msg(self, msg: str):
        self.get_logger().error(msg)


def main(argv=sys.argv[1:]):
    rclpy.init()

    starting_pose = [-2.0, -0.5]

    test = WaypointFollowerTest()

    retry_count = 0
    retries = 2000
    while not test.initial_pose_received and retry_count <= retries:
        retry_count += 1
        test.info_msg('Setting initial pose')
        test.setInitialPose(starting_pose)
        test.info_msg('Waiting for amcl_pose to be received')
        rclpy.spin_once(test, timeout_sec=1.0)  # wait for poseCallback

    while test.costmap == None:
        test.info_msg('Getting initial map')
        rclpy.spin_once(test, timeout_sec=1.0)

    test.moveToFrontiers()

    rclpy.spin(test)


if __name__ == '__main__':
    main()