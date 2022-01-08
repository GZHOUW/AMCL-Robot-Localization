#!/usr/bin/env python3

# BEFORE RUNNING THIS FILE, RUN robot_start.launch

import threading

import rospy
import tf.transformations as tr

from std_srvs.srv import Trigger, TriggerRequest
from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import JointState
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from visualization_msgs.msg import Marker, MarkerArray

from smach import State, StateMachine, Sequence
from smach_ros import (
    ServiceState,
    SimpleActionState,
    ActionServerWrapper,
    IntrospectionServer,
)

import hello_helpers.hello_misc as hm
import stretch_funmap.navigate as nv

from sp_core.custom_states import *
from sp_msgs.srv import Order, OrderRequest, OrderResponse

SHELF_GOAL_M = ([0.0, 0.1, 0.0], 0.0)
WORKSTATION_GOAL_M = ([1.3, 2.4, 0.0], 1.57)


class StateMachineNode(hm.HelloNode):
    def __init__(self):

        hm.HelloNode.__init__(self)

        self.rate = 10.0
        self.joint_states = None
        self.joint_states_lock = threading.Lock()
        self.move_base = nv.MoveBase(self)
        self.wait_for_first_pointcloud = False
        self.wrist_position = None
        self.lift_position = None

        self.shelf_goal = ([0.0, 0.0, 0.0], 0.0)
        self.workstation_goal = ([1.3, 2.4, 0.0], 1.57)
        self.marker_list = [80]
        self.is_start = False

        self.vis_marker_array = MarkerArray()

        self.vis_marker_publisher = rospy.Publisher(
            "debug_vis_markers", MarkerArray, queue_size=10
        )

        self.current_order_publisher = rospy.Publisher(
            "sp_sm/current_orders", Int16MultiArray, queue_size=10
        )

        self.receive_order_subscriber = rospy.Subscriber(
            "sp_sm/post_orders", Int16MultiArray, self.receive_order_callback
        )

        self.trigger_magnet_service = rospy.ServiceProxy(
            "/toggle_magnet", Trigger
        )

        self.trigger_clear_orders_service = rospy.Service(
            "sp_sm/clear_orders",
            Trigger,
            self.trigger_clear_orders_service_callback,
        )

        self.trigger_start_orders_service = rospy.Service(
            "sp_sm/start",
            Trigger,
            self.trigger_start_execution_service_callback,
        )

    def receive_order_callback(self, msg):
        rospy.loginfo(f"{len(msg.data)} orders received: {msg.data}.")
        for order in msg.data:
            if not int(order) in self.marker_list:
                self.marker_list.append(int(order))

        curr_orders = Int16MultiArray()
        curr_orders.data = self.marker_list
        self.current_order_publisher.publish(curr_orders)

    def trigger_clear_orders_service_callback(self, req):
        res = OrderResponse()
        res.success = True
        res.message = "Orders cleared."
        self.marker_list = []

        curr_orders = Int16MultiArray()
        curr_orders.data = self.marker_list
        self.current_order_publisher.publish(curr_orders)

        return res

    def trigger_start_execution_service_callback(self, req):
        res = OrderResponse()
        res.success = True
        res.message = "Execution start."
        self.is_start = True

        curr_orders = Int16MultiArray()
        curr_orders.data = self.marker_list
        self.current_order_publisher.publish(curr_orders)

        return res

    def make_move_base_goal(self, position, orientation):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = position[0]
        goal.target_pose.pose.position.y = position[1]
        goal.target_pose.pose.position.z = 0.0
        if isinstance(orientation, list):
            if len(orientation) == 3:
                orientation = tr.quaternion_from_euler(*orientation)
        elif isinstance(orientation, float):
            orientation = tr.quaternion_from_euler(0.0, 0.0, orientation)
        goal.target_pose.pose.orientation.x = orientation[0]
        goal.target_pose.pose.orientation.y = orientation[1]
        goal.target_pose.pose.orientation.z = orientation[2]
        goal.target_pose.pose.orientation.w = orientation[3]

        return goal

    def update_vis_markers(self, frame_id, postion, duration=0.0):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = postion[0]
        marker.pose.position.y = postion[1]
        marker.pose.position.z = postion[2]
        # marker.lifetime = rospy.Duration.from_sec(duration)

        if len(self.vis_marker_array.markers) >= 2:
            self.vis_marker_array.markers.pop(0)
        self.vis_marker_array.markers.append(marker)

        self.vis_marker_publisher.publish(self.vis_marker_array)

    def joint_states_callback(self, joint_states):
        with self.joint_states_lock:
            self.joint_states = joint_states
        wrist_position, wrist_velocity, wrist_effort = hm.get_wrist_state(
            joint_states
        )
        self.wrist_position = wrist_position
        lift_position, lift_velocity, lift_effort = hm.get_lift_state(
            joint_states
        )
        self.lift_position = lift_position

    def lookup_transform_mat(self, base_frame, target_frame):
        mat, _ = hm.get_p1_to_p2_matrix(
            target_frame, base_frame, self.tf2_buffer
        )
        return mat

    def grasp_fsm(self):
        sm = StateMachine(outcomes=["succeeded", "aborted", "preempted"])
        with sm:
            StateMachine.add(
                "MODE_POSITION",
                ServiceState("/switch_to_position_mode", Trigger),
                transitions={"succeeded": "PRESEARCH"},
            )
            StateMachine.add(
                "PRESEARCH",
                PreSearchState(self),
                transitions={
                    "succeeded": "SEARCH_MARKERS",
                    "aborted": "PRESEARCH",
                },
            )
            StateMachine.add(
                "SEARCH_MARKERS",
                SearchMarkersState(self.marker_list),
                transitions={
                    "detected": "PREDETECT",
                    "undetected": "SEARCH_MARKERS",
                },
                remapping={"detected_marker_id": "detected_marker_id"},
            )
            StateMachine.add(
                "PREDETECT",
                PreDetectState(self),
                transitions={
                    "succeeded": "DETECT",
                },
                remapping={"detected_marker_id": "detected_marker_id"},
            )

            StateMachine.add(
                "DETECT",
                DetectState(self),
                transitions={"succeeded": "GRASP", "aborted": "aborted"},
                remapping={
                    "detected_marker_id": "detected_marker_id",
                    "target": "target",
                    "target_frame": "target_frame",
                },
            )

            StateMachine.add(
                "GRASP",
                GraspCupState(self),
                transitions={"succeeded": "OLID"},
            )

            StateMachine.add(
                "OLID",
                OpenLidState(self),
                transitions={"succeeded": "POD"},
            )
            
            StateMachine.add(
                "POD",
                InsertPodState(self),
                transitions={"succeeded": "CLID"},
            )
  
            StateMachine.add(
                "CLID",
                CloseLidState(self),
                transitions={"succeeded": "BUTTON"},
            )

            StateMachine.add(
                "BUTTON",
                PushButtonState(self),
                transitions={"succeeded": "succeeded"},
            )
        
        return sm

    def main(self):
        hm.HelloNode.main(
            self,
            "sp_state_machine",
            "sp_state_machine",
            wait_for_first_pointcloud=False
        )

        self.vis_marker_publisher.publish(self.vis_marker_array)

        self.joint_states_subscriber = rospy.Subscriber(
            "/stretch/joint_states", JointState, self.joint_states_callback
        )

        service_name = "/switch_to_position_mode"
        rospy.wait_for_service(service_name)
        rospy.loginfo(
            f"Node {self.node_name} connected to {service_name} service."
        )

        self.switch_to_position_mode_service = rospy.ServiceProxy(
            service_name, Trigger
        )

        sm = self.grasp_fsm()

        sis = IntrospectionServer("state_machine", sm, "/state_machine")
        sis.start()

        # Execute SMACH plan
        outcome = sm.execute()

        # Wait for ctrl-c to stop the application
        rospy.spin()


if __name__ == "__main__":
    try:
        # parser = ap.ArgumentParser(description="Handover an object.")
        # args, unknown = parser.parse_known_args()
        node = StateMachineNode()
        node.main()
    except KeyboardInterrupt:
        rospy.loginfo("interrupt received, so shutting down")
