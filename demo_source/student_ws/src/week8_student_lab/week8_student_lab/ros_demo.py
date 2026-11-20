#!/usr/bin/env python3
"""RViz-only teaching simulation (not Gazebo or Nav2-controller motion).

Five modes: map, plan, move, mission, explore.
Uses ONLY Python ROS 2 messages plus our pedagogical 4-connected A*.
It deliberately does not publish /cmd_vel or claim a real SLAM/vision sensor.
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster
from .world import W,H,RES,START,BOX,TOYS,WALLS,center
from .search import astar,frontiers
from .mission import MissionModel


def rgba(r,g,b,a=1.0):
    from std_msgs.msg import ColorRGBA
    col=ColorRGBA(); col.r=r;col.g=g;col.b=b;col.a=a
    return col


class Demo(Node):
    def __init__(self):
        super().__init__('week8_student_simulation')
        self.declare_parameter('mode','map')
        self.declare_parameter('step_period',0.45)
        self.mode=self.get_parameter('mode').value
        if self.mode not in ('map','plan','move','mission','explore'):
            raise ValueError('mode must be map, plan, move, mission, or explore')
        self.step_period=max(0.12,float(self.get_parameter('step_period').value))
        latch=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.map_pub=self.create_publisher(OccupancyGrid,'/student/map',latch)
        self.path_pub=self.create_publisher(Path,'/student/path',latch)
        self.marker_pub=self.create_publisher(MarkerArray,'/student/markers',latch)
        self.odom_pub=self.create_publisher(Odometry,'/student/odom',10)
        self.tf=TransformBroadcaster(self)
        self.model=MissionModel('explore' if self.mode=='explore' else 'mission') if self.mode in ('mission','explore') else None
        self.pose=START
        self.path=[] if self.mode=='map' else astar(START,BOX)[0]
        self.pos_index=0
        self.done_announced=False
        self.timer=self.create_timer(self.step_period,self.tick)
        self.publish_all()
        self.get_logger().info(f'Week 8 student demo: MODE={self.mode}; Fixed Frame=map; topics: /student/map /student/path /student/markers /student/odom')
        self.get_logger().warn('Teaching only: 2D marker animation. NO Nav2 planner server, true SLAM, physics or hardware driving.')

    def make_pose(self,cell,stamp):
        p=PoseStamped()
        p.header.frame_id='map';p.header.stamp=stamp
        p.pose.position.x,p.pose.position.y=center(cell)
        p.pose.orientation.w=1.0
        return p

    def publish_all(self):
        stamp=self.get_clock().now().to_msg()
        known=(self.model.known if self.model else
               {(x,y):(100 if (x,y) in WALLS else 0) for y in range(H) for x in range(W)})
        pose=(self.model.position if self.model else self.pose)
        route=(self.model.route if self.model else self.path[self.pos_index:])
        # Map: row-major, bottom-left grid origin, -1=UNKNOWN, 0=FREE, 100=OCCUPIED.
        grid=OccupancyGrid()
        grid.header.frame_id='map';grid.header.stamp=stamp
        grid.info.width=W;grid.info.height=H;grid.info.resolution=RES
        grid.info.origin.orientation.w=1.0
        grid.data=[known.get((x,y),-1) for y in range(H) for x in range(W)]
        self.map_pub.publish(grid)
        # Path contains a sequence of PoseStamped world coordinates.
        path=Path()
        path.header.frame_id='map';path.header.stamp=stamp
        path.poses=[self.make_pose(c,stamp) for c in route]
        self.path_pub.publish(path)
        # Markers work even in RViz instances where the Map shader is broken.
        markers=MarkerArray()
        erase=Marker();erase.header.frame_id='map';erase.header.stamp=stamp;erase.action=Marker.DELETEALL
        markers.markers.append(erase)
        obs=Marker();obs.header.frame_id='map';obs.header.stamp=stamp;obs.ns='walls';obs.id=10
        obs.type=Marker.CUBE_LIST;obs.action=Marker.ADD
        obs.scale.x=RES;obs.scale.y=RES;obs.scale.z=0.12;obs.color=rgba(.15,.23,.33,1.0)
        from geometry_msgs.msg import Point
        for c,v in known.items():
            if v==100:
                pt=Point();pt.x,pt.y=center(c);pt.z=0.06
                obs.points.append(pt)
        markers.markers.append(obs)
        def blob(name,id_,where,scale,color):
            m=Marker();m.header.frame_id='map';m.header.stamp=stamp;m.ns=name;m.id=id_
            m.type=Marker.SPHERE;m.action=Marker.ADD;m.pose.position.x,m.pose.position.y=center(where)
            m.pose.position.z=.10;m.pose.orientation.w=1.0
            m.scale.x=m.scale.y=m.scale.z=scale;m.color=color;markers.markers.append(m)
        blob('robot',1,pose,0.18,rgba(.08,.50,1.0))
        if self.mode in ('map','plan','move'):
            blob('goal',2,BOX,0.21,rgba(.1,.82,.37))
        else:
            for i,t in enumerate(TOYS):
                if t in self.model.remaining and (self.mode=='mission' or t in self.model.seen_toys):
                    blob('toy',20+i,t,.18,rgba(1.0,.58,.12))
            if self.mode=='mission' or self.model.box_seen:
                blob('box',30,BOX,.25,rgba(.12,.8,.30))
            if self.model.goal is not None:
                blob('temporary_goal',40,self.model.goal,.11,rgba(.75,.18,.90))
        if self.mode=='explore':
            f=Marker();f.header.frame_id='map';f.header.stamp=stamp;f.ns='frontiers';f.id=50
            f.type=Marker.SPHERE_LIST;f.action=Marker.ADD
            f.scale.x=f.scale.y=f.scale.z=.07;f.color=rgba(.95,.77,.11)
            for c in frontiers(known):
                pt=Point();pt.x,pt.y=center(c);pt.z=.08;f.points.append(pt)
            markers.markers.append(f)
        # A status label that remains above the robot and is easy to discuss.
        m=Marker();m.header.frame_id='map';m.header.stamp=stamp;m.ns='status';m.id=100
        m.type=Marker.TEXT_VIEW_FACING;m.action=Marker.ADD
        m.pose.position.x,m.pose.position.y=center(pose);m.pose.position.z=.35;m.pose.orientation.w=1.0
        m.scale.z=.14;m.color=rgba(.10,.10,.10)
        if self.model:
            m.text=f'{self.model.phase} | delivered={self.model.delivered}/{len(TOYS)}'
        else:
            m.text=self.mode.upper()
        markers.markers.append(m)
        self.marker_pub.publish(markers)
        # Odometry/TF are PERFECT synthetic poses, not estimated localization.
        odom=Odometry();odom.header.frame_id='map';odom.header.stamp=stamp
        odom.child_frame_id='base_link';odom.pose.pose.position.x,odom.pose.pose.position.y=center(pose)
        odom.pose.pose.orientation.w=1.0
        self.odom_pub.publish(odom)
        tf=TransformStamped();tf.header.frame_id='map';tf.header.stamp=stamp;tf.child_frame_id='base_link'
        tf.transform.translation.x,tf.transform.translation.y=center(pose)
        tf.transform.rotation.w=1.0
        self.tf.sendTransform(tf)

    def tick(self):
        if self.mode=='move':
            if self.pos_index<len(self.path)-1:
                self.pos_index+=1
                self.pose=self.path[self.pos_index]
            elif not self.done_announced:
                self.get_logger().info('Reached goal (educational marker animation; not actual motor control)')
                self.done_announced=True
        elif self.model:
            previous=self.model.last_event
            message=self.model.step()
            if message!=previous:
                self.get_logger().info(message)
            if self.model.phase in ('DONE','FAILED') and not self.done_announced:
                self.get_logger().info('FINAL: '+self.model.phase)
                self.done_announced=True
        self.publish_all()


def main(args=None):
    rclpy.init(args=args)
    node=None
    try:
        node=Demo()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__=='__main__':
    main()
