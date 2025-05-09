import gymnasium as gym
from gymnasium import spaces
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge
from nav_msgs.msg import Path, Odometry

import cv2
from scipy.spatial.transform import Rotation
import subprocess


class TrailerLoadEnv(gym.Env):
    """Gymnasium Environment for WAMV in Gazebo with ROS2"""

    metadata = {'render_modes': ['human', 'rgb_array'], 'render_fps': 30}

    def __init__(self, render_mode=None):
        super(TrailerLoadEnv, self).__init__()

        # Initialize ROS2 if not already done
        if not rclpy.ok():
            rclpy.init()

        # Create ROS2 node
        self.node = Node('wamv_gym_wrapper')
        self.bridge = CvBridge()
        self.render_mode = render_mode
        self.episode_max_step = 100
        self.episope_cnt = 0

        # reward weights (tune these based on task)
        self.w_pos = 1.0         # weight for position error
        self.w_yaw = 0.5         # weight for yaw error
        self.w_success = 100.0   # reward for successful docking
        self.w_collision = -100.0 # penalty for collision
        self.pos_threshold = 0.5 # meters
        self.yaw_threshold = np.deg2rad(10) # radians
        self.collision = False

        # Current state
        self.x = 0
        self.y = 0
        self.yaw = 0
        self.u = 0
        self.v = 0
        self.r = 0

        # Goal state
        self.x_goal = -51.36
        self.y_goal = -153.81
        self.yaw_goal = -1.57

        # Define action and observation space
        self.action_space = spaces.Box(
            low=np.array([-1.0, -1.0]),  # Normalized thrust values [-1, 1]
            high=np.array([1.0, 1.0]),
            dtype=np.float32)

        # Observation space is the camera image
        # Assuming 640x480 RGB image
        self.observation_space = spaces.Box(
            low=0,
            high=255,
            shape=(480, 640, 3),  # Height, Width, Channels
            dtype=np.uint8)

        # Setup ROS2 communications
        self._setup_ros_communications()

        # Initialize state variables
        self.current_image = None
        self.episode_done = False
        self.received_image = False

        self.loading = False

        # For rendering
        if self.render_mode == 'human':
            cv2.namedWindow('WAMV Camera View', cv2.WINDOW_NORMAL)

    def _setup_ros_communications(self):
        """Set up all ROS2 publishers and subscribers"""
        # Subscriber for camera image
        self.image_sub = self.node.create_subscription(
            Image,
            '/wamv/sensors/cameras/front_camera_sensor/image_raw',
            self._image_callback,
            10)

        # Publishers for thrust commands
        self.left_thrust_pub = self.node.create_publisher(
            Float64,
            '/wamv/thrusters/left/thrust',
            10)

        self.right_thrust_pub = self.node.create_publisher(
            Float64,
            '/wamv/thrusters/right/thrust',
            10)
        self.fix_vel_sub = self.node.create_subscription(Odometry, '/wamv/sensors/position/ground_truth_odometry',
                                                    self._get_vel, 5)
        # Optional: Service client for reset if available

    def _image_callback(self, msg):
        """Callback for processing camera images"""
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.current_image = cv_image
            self.received_image = True

            # Update render if in human mode
            if self.render_mode == 'human':
                self.render()
        except Exception as e:
            self.node.get_logger().error(f'Error processing image: {str(e)}')

    def _get_vel(self, data):
        """
        Gazebo publishes, update robot heading, x and y velocities in robot (local) frame
        Gps fix velocity is in the global coordinate frame that rotates 90 degrees counter-clockwise of the
        world (ENU) coordinate frame
        """
        # if not self.state_ready:
        #     self.state_ready = True
        # else:
        #     self.get_logger().info('State is ready.', once=True)

        self.y = data.pose.pose.position.y  # + np.random.normal(0, 0.1)
        self.x = data.pose.pose.position.x  # + np.random.normal(0, 0.1)
        data_imu = data.pose.pose
        q = [data_imu.orientation.x, data_imu.orientation.y, data_imu.orientation.z, data_imu.orientation.w]

        _, _, self.yaw = Rotation.from_quat(q).as_euler('xyz')
        # self.state.yaw += np.random.normal(0, 0.01)
        data_vel = data.twist.twist
        # To robot frame
        self.u = data_vel.linear.x
        self.v = data_vel.linear.y
        self.r = data_vel.angular.z
        # print("here")

    def step(self, action):
        """
        Execute one time step in the environment

        Args:
            action: [left_thrust, right_thrust] normalized between -1 and 1

        Returns:
            observation (image), reward, terminated, truncated, info
        """
        # Publish the thrust commands
        self.episope_cnt += 1
        left_thrust = Float64()
        left_thrust.data = float(action[0])
        self.left_thrust_pub.publish(left_thrust)

        right_thrust = Float64()
        right_thrust.data = float(action[1])
        self.right_thrust_pub.publish(right_thrust)

        # Wait for new image observation
        self.received_image = False
        start_time = self.node.get_clock().now()
        while not self.received_image:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            # Timeout after 2 seconds
            if (self.node.get_clock().now() - start_time).nanoseconds > 2e9:
                self.node.get_logger().warn('Timeout waiting for image observation')
                self.episode_done = True
                break



        # Calculate reward (implement your own reward function)
        reward = self._calculate_reward()
        if self.episope_cnt > self.episode_max_step or self.loading:
            self.episode_done = True
        # Check if episode is done
        terminated = self.episode_done
        truncated = False  # You can set this if you have a max episode length
        if terminated:
            self.episode_done = False  # Reset for next episode

        # Additional info (optional)
        info = {
            'left_thrust': action[0],
            'right_thrust': action[1]
        }

        return self.current_image, reward, terminated, truncated, info

    def reset(self, seed=None, options=None):
        """
        Reset the environment to initial state

        Args:
            seed: Optional seed for random number generator
            options: Optional additional options

        Returns:
            observation (image) and info
        """
        # We don't need the seed for this environment
        super().reset(seed=seed)

        # Publish zero thrust commands first
        zero_thrust = Float64()
        zero_thrust.data = 0.0
        self.left_thrust_pub.publish(zero_thrust)
        self.right_thrust_pub.publish(zero_thrust)
        self.loading = False
        # Call reset service

        self.episope_cnt = 0
        x = -51.5
        y = -150
        yaw = -1.58
        _, _, z, w = Rotation.from_euler('xyz', [0, 0, yaw]).as_quat()
        command = [
            "gz", "service",
            "-s", "/world/lake_harner_trailer/set_pose",
            "--reqtype", "gz.msgs.Pose",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "1000",
            "--req", f'name: "wamv", position: {{x: {x}, y: {y}, z: 0}}, orientation: {{x: 0, y: 0, z: {z},w: {w}}}'
        ]

        try:
            # Run the command
            result = subprocess.run(command, capture_output=True, text=True, check=True)
            # Print the result of the service call
            print("Reset successful")
            # print("Output:", result.stdout)
            # print("Error (if any):", result.stderr)
        except subprocess.CalledProcessError as e:
            print(f"Error occurred: {e}")
            print("Output:", e.output)
            print("Error:", e.stderr)

        # Wait for new image observation
        self.received_image = True
        start_time = self.node.get_clock().now()
        while not self.received_image:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            # Timeout after 2 seconds
            if (self.node.get_clock().now() - start_time).nanoseconds > 2e9:
                self.node.get_logger().error('Timeout waiting for image during reset')
                raise RuntimeError('Failed to receive image observation during reset')
        info = {}  # Can add reset-specific info here if needed
        return self.current_image, info

    def render(self):
        """Render the environment"""
        if self.render_mode == 'rgb_array':
            return self.current_image
        elif self.render_mode == 'human':
            if self.current_image is not None:
                cv2.imshow('WAMV Camera View', self.current_image)
                cv2.waitKey(1)
            return
        # No return value needed for human mode

    def close(self):
        """Clean up resources"""
        # Publish zero thrust before closing
        zero_thrust = Float64()
        zero_thrust.data = 0.0
        self.left_thrust_pub.publish(zero_thrust)
        self.right_thrust_pub.publish(zero_thrust)

        # Destroy node and shutdown ROS
        self.node.destroy_node()
        rclpy.shutdown()

        # Close any OpenCV windows
        if self.render_mode == 'human':
            cv2.destroyAllWindows()

    def _calculate_reward(self):
        """Implement your custom reward function here"""
        # Compute position error
        dx = self.x_goal - self.x
        dy = self.y_goal - self.y
        pos_error = np.sqrt(dx ** 2 + dy ** 2)

        # Compute yaw error
        yaw_error = np.arctan2(np.sin(self.yaw_goal - self.yaw),
                               np.cos(self.yaw_goal - self.yaw))  # normalized

        # Shaped negative reward for error
        reward = -self.w_pos * pos_error - self.w_yaw * abs(yaw_error)

        # Check for success
        if pos_error < self.pos_threshold and abs(yaw_error) < self.yaw_threshold:
            reward += self.w_success
            self.loading = True
        # Check for collision
        if self.collision:
            reward += self.w_collision

        print(f"[DEBUG] pos_error: {pos_error:.3f} m, yaw_error: {np.rad2deg(yaw_error):.2f} deg, reward: {reward:.2f}")

        return reward