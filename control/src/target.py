import gymnasium as gym
from gymnasium import spaces

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64

from cv_bridge import CvBridge
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
import subprocess
from typing import Optional, Tuple, Dict, Any


class WamvGazeboEnv(gym.Env):
    """Gymnasium Environment for WAMV in Gazebo with ROS2"""

    metadata = {'render_modes': ['human', 'rgb_array'], 'render_fps': 30}

    def __init__(
            self,
            img_height: int = 480,
            img_width: int = 640,
            incremental_action: bool = True,
            episode_max_step: int = 1000,
            obs_timeout_sec: float = 2.,
            render_mode: Optional[str] = None,
    ):
        """
        Args:
            img_height: Desired image height from onboard camera.
            img_width: Desired image width from onboard camera.
            incremental_action: Whether accept delta thrust or absolute thrust in step function.
            episode_max_step: Maximum number of steps in an episode.
            obs_timeout_sec: Timeout seconds waiting for camera topics.
            render_mode: Render mode, {'rgb_array', 'human'}.
        """
        super(WamvGazeboEnv, self).__init__()

        # Initialize ROS2 if not already done
        if not rclpy.ok():
            rclpy.init()

        # Create ROS2 node
        self.node = Node('wamv_gym_wrapper')
        self.bridge = CvBridge()
        self.render_mode = render_mode
        self.episode_max_step: int = episode_max_step
        self.obs_timeout_sec: float = obs_timeout_sec
        self.episode_step: int = 0
        self.img_height: int = img_height
        self.img_width: int = img_width
        self.incremental_action: bool = incremental_action

        # Define action and observation space
        self.action_space = spaces.Box(
            low=np.array([-1.0, -1.0]),  # Normalized thrust values [-1, 1]
            high=np.array([1.0, 1.0]),
            dtype=np.float64)

        # Observation space is the camera image
        # Assuming 640x480 RGB image
        self.observation_space = spaces.Tuple((
            # For RGB image
            spaces.Box(
                low=0,
                high=255,
                shape=(img_height, img_width, 3),  # Height, Width, Channels
                dtype=np.uint8),
            # For semantic mask
            spaces.Box(
                low=0,
                high=255,
                shape=(img_height, img_width, 3),  # Height, Width, Channels
                dtype=np.uint8),
        ))

        # Setup ROS2 communications
        self._setup_ros_communications()

        # Initialize state variables
        self.current_image: Optional[np.ndarray] = None
        self.received_image: bool = False
        self.current_mask: Optional[np.ndarray] = None
        self.receive_mask: bool = False
        self.current_action: np.array = np.array([0., 0.])

        # For rendering
        if self.render_mode == 'human':
            cv2.namedWindow('WAMV Camera View', cv2.WINDOW_NORMAL)

    def _setup_ros_communications(self):
        """Set up all ROS2 publishers and subscribers"""
        # Subscriber for camera image
        self.image_sub = self.node.create_subscription(
            Image,
            '/wamv/sensors/cameras/front_left_camera_sensor/image_raw',
            self._image_callback,
            10)

        # Subscriber for camera mask
        self.mask_sub = self.node.create_subscription(
            Image,
            '/front_left_camera_segmentation/colored_map',  # Topic of segmentation camera
            self._mask_callback,
            10,
        )

        # Publishers for thrust commands
        self.left_thrust_pub = self.node.create_publisher(
            Float64,
            '/wamv/thrusters/left/thrust',
            10)

        self.right_thrust_pub = self.node.create_publisher(
            Float64,
            '/wamv/thrusters/right/thrust',
            10)

        # Optional: Service client for reset if available

    def _image_callback(self, msg: Image) -> None:
        """Callback for processing camera images"""
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv_image= cv2.resize(cv_image, (self.img_width, self.img_height), interpolation=cv2.INTER_LINEAR)
            self.current_image = cv_image
            self.received_image = True
        except Exception as e:
            self.node.get_logger().error(f'Error processing image: {str(e)}')

    def _mask_callback(self, msg: Image) -> None:
        try:
            cv_mask = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv_mask = cv2.cvtColor(cv_mask, cv2.COLOR_BGR2GRAY)
            cv_mask = cv2.resize(cv_mask, (self.img_width, self.img_height), interpolation=cv2.INTER_NEAREST)

            # Post-process binary mask
            cv_mask[cv_mask == 255] = 0  # Black
            cv_mask[cv_mask != 0] = 255  # White

            self.current_mask = cv_mask
            self.receive_mask = True

            # unique_values = np.unique(self.current_mask)
            # self.node.get_logger().info(f'Mask {unique_values=}')
        except Exception as e:
            self.node.get_logger().error(f'Error processing mask: {str(e)}')

    def pub_thrust(self, left: float = 0., right: float = 0.) -> None:
        """
        Publish thruster commands for left and right thrusters.
        Thrust value ranges in [-1000, 1000]
        """
        if abs(left) > 1000.:
            self.node.get_logger().warn(f'Left thrust cmd needs to be in range [-1000, 1000], given {left}.')
        if abs(right) > 1000.:
            self.node.get_logger().warn(f'Right thrust cmd needs to be in range [-1000, 1000], given {right}.')

        # Publish the thrust commands
        left_thrust = Float64()
        left_thrust.data = float(left)
        self.left_thrust_pub.publish(left_thrust)

        right_thrust = Float64()
        right_thrust.data = float(right)
        self.right_thrust_pub.publish(right_thrust)

    def wait_for_obs(self) -> bool:
        """
        Wait for specified duration until image observation is available.

        Returns:
            Whether the observation is available before timeout.
        """
        self.received_image = False
        self.receive_mask = False
        start_time = self.node.get_clock().now()
        while not self.received_image or not self.receive_mask:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if (self.node.get_clock().now() - start_time).nanoseconds > self.obs_timeout_sec * 1e9:
                self.node.get_logger().error('Timeout waiting for image during reset.')
                return False

        return True

    def step(self, action: np.array) -> Tuple[Tuple[np.ndarray, np.ndarray], float, bool, bool, Dict[str, Any]]:
        """
        Execute one time step in the environment

        Args:
            action: [left_thrust, right_thrust] normalized between -1 and 1

        Returns:
            observation (image, mask), reward, terminated, truncated, info
        """
        self.episode_step += 1

        assert np.all(np.abs(action) <= 1), f'Agent action should range in [-1, 1], given {action}.'

        a = action.copy()

        # Increment the action instead of setting absolute values
        if self.incremental_action:
            a += self.current_action
            a = np.clip(a, a_min=-1., a_max=1.)
            np.copyto(self.current_action, a)

        # Scale action to thrust range of wamv
        a *= 1000

        # Publish the thrust commands
        self.pub_thrust(left=a[0], right=a[1])

        terminated: bool = False
        truncated: bool = False

        # Wait for new image observation
        if not self.wait_for_obs():
            terminated = True

        # Max episodic steps reached
        if self.episode_step > self.episode_max_step:
            truncated = True

        # Update render if in human mode
        if self.render_mode == 'human':
            self.render()

        # Calculate reward (implement your own reward function)
        reward = self._calculate_reward()

        # Additional info (optional)
        info = {
            'left_thrust': action[0],
            'right_thrust': action[1]
        }

        return (self.current_image, self.current_mask), reward, terminated, truncated, info

    def reset(self, seed=None, options=None) -> Tuple[Tuple[np.ndarray, np.ndarray], Dict[str, Any]]:
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
        self.pub_thrust()
        self.current_action = np.array([0., 0.])
        self.episode_step = 0

        # Call reset service
        x = -10
        y = -1000
        yaw = 1.14
        _, _, z, w = Rotation.from_euler('xyz', [0, 0, yaw]).as_quat()
        command = [
            "gz", "service",
            "-s", "/world/wabash/set_pose",
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
        if not self.wait_for_obs():
            raise RuntimeError('Failed to receive image observation during reset.')

        info = {}  # Can add reset-specific info here if needed
        return (self.current_image, self.current_mask), info

    def render(self) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """Render the environment"""
        if self.render_mode == 'rgb_array':
            return self.current_image, self.current_mask
        elif self.render_mode == 'human':
            if self.current_image is not None and self.current_mask is not None:
                mask_3ch = np.repeat(self.current_mask[:, :, np.newaxis], 3, axis=-1)
                img_mask = cv2.hconcat([self.current_image, mask_3ch])
                cv2.imshow('WAMV Camera View', img_mask)
                cv2.waitKey(1)
            else:
                self.node.get_logger().warn('Waiting for both rgb and mask to be available to render.')
        else:
            self.node.get_logger().warn(f'Unrecognized render mode {self.render_mode}.')

    def close(self) -> None:
        """Clean up resources"""
        # Publish zero thrust before closing
        self.pub_thrust()

        # Destroy node and shutdown ROS
        self.node.destroy_node()
        rclpy.shutdown()

        # Close any OpenCV windows
        if self.render_mode == 'human':
            cv2.destroyAllWindows()

    def _calculate_reward(self) -> float:
        """Implement your custom reward function here"""
        # Example: simple reward for moving forward
        # You'll want to replace this with your actual reward logic
        return 1.0  # Placeholder