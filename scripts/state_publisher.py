from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped

from time import sleep
from copy import copy


path = [
    { #   --------------- 1 -----------------
        "t":3,
        "arm":[169,60,-140,100,167]
    },{
        "t":3,
        "arm":[-45,7,-7,7,7]
    },{
        "t":3,
        "arm":[-45,60,-140,145,100]
    },{
        "t":3,
        "arm":[-45,7,-7,7,7]
    },{
        "t":3,
        "arm":[-45,60,-140,145,167]
    },{ # --------------- 2 -----------------
        "t":6,
        "arm":[169,60,-140,100,100]
    },{ # --------------- 3 -----------------
        "t":4,
        "arm":[169,60,-100,100,167]
    },{
        "t":3,
        "arm":[7,60,-100,100,167]
    },{
        "t":3,
        "arm":[169,60,-100,100,167]
    },{
        "t":3,
        "arm":[7,60,-100,100,167]
    },{
        "t":5,
        "arm":[169,60,-140,100,167]
    },{ # --------------- 4 -----------------
        "t":3,
        "arm":[7,7,-7,7,7]
    },{
        "t":10,
        "arm":[7,7,-7,7,7]
    }
]


class StatePublisher(Node):
    
    def smother(self, d, x0, x1):
        return float(x0)+(float(x1-x0)*d)

    def __init__(self):
        rclpy.init()
        super().__init__('state_publisher')

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        degree = pi / 180.0
        loop_rate = self.create_rate(30)

        # robot state
        virtual_joint_x = 0.
        virtual_joint_y = 0.
        virtual_joint_z = 0.
        wheel_joint_fl = 0.
        wheel_joint_fr = 0.
        wheel_joint_bl = 0.
        wheel_joint_br = 0.
        arm_joint_1 = 0.
        arm_joint_2 = 0.
        arm_joint_3 = 0.
        arm_joint_4 = 0.
        arm_joint_5 = 0.

        # message declarations
        joint_state = JointState()

        try:
            i = 0
            while rclpy.ok():
                rclpy.spin_once(self)

                # update joint_state
                now = self.get_clock().now()
                joint_state.header.stamp = now.to_msg()
                joint_state.name = [
                    'virtual_joint_x',
                    'virtual_joint_y',
                    'virtual_joint_z',
                    'wheel_joint_fl',
                    'wheel_joint_fr',
                    'wheel_joint_bl',
                    'wheel_joint_br',
                    'arm_joint_1',
                    'arm_joint_2',
                    'arm_joint_3',
                    'arm_joint_4',
                    'arm_joint_5'
                ]
                joint_state.position = [
                    virtual_joint_x,
                    virtual_joint_y,
                    virtual_joint_z,
                    wheel_joint_fl,
                    wheel_joint_fr,
                    wheel_joint_bl,
                    wheel_joint_br,
                    arm_joint_1,
                    arm_joint_2,
                    arm_joint_3,
                    arm_joint_4,
                    arm_joint_5
                ]

                # send the joint state and transform
                self.joint_pub.publish(joint_state)

                print(1)

                if i < len(path):
                    if "arm" in path[i].keys():
                        print(2)

                        t0 = now.nanoseconds

                        arm_joint_1_, arm_joint_2_, arm_joint_3_, arm_joint_4_, arm_joint_5_ = arm_joint_1, arm_joint_2, arm_joint_3, arm_joint_4, arm_joint_5

                        while (now.nanoseconds-t0) < path[i]["t"]*(10 ** 9):
                            rclpy.spin_once(self)
                            now = self.get_clock().now()
                            d = (now.nanoseconds-t0)/(path[i]["t"]*(10 ** 9))
                            print(d)
                            
                            joint_state.header.stamp = now.to_msg()
                            arm_joint_1 = self.smother(d, arm_joint_1_, degree*path[i]["arm"][0])
                            arm_joint_2 = self.smother(d, arm_joint_2_, degree*path[i]["arm"][1])
                            arm_joint_3 = self.smother(d, arm_joint_3_, degree*path[i]["arm"][2])
                            arm_joint_4 = self.smother(d, arm_joint_4_, degree*path[i]["arm"][3])
                            arm_joint_5 = self.smother(d, arm_joint_5_, degree*path[i]["arm"][4])


                            joint_state.position = [
                                virtual_joint_x,
                                virtual_joint_y,
                                virtual_joint_z,
                                wheel_joint_fl,
                                wheel_joint_fr,
                                wheel_joint_bl,
                                wheel_joint_br,
                                arm_joint_1,
                                arm_joint_2,
                                arm_joint_3,
                                arm_joint_4,
                                arm_joint_5
                            ]

                            self.joint_pub.publish(joint_state)

                            loop_rate.sleep()

                        #sleep(path[i]["t"])
                else:
                    return

                i += 1

                # This will adjust as needed per iteration
                loop_rate.sleep()

        except KeyboardInterrupt:
            pass

def main():
    node = StatePublisher()

if __name__ == '__main__':
    main()