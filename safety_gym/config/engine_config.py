from dataclasses import dataclass
from typing import List
from typing import Optional


@dataclass()
class EngineConfig:
    _target_: str = "safety_gym.envs.engine.Engine"
    num_steps: int = 1000  # Maximum number of environment steps in an episode

    action_noise: float = 0.0  # Magnitude of independent per-component gaussian action noise

    placements_extents: List[int] = (-2, -2, 2, 2)  # Placement limits (min X min Y max X max Y)
    placements_margin: float = 0.0  # Additional margin added to keepout when placing objects

    # Floor
    floor_display_mode: bool = False  # In display mode the visible part of the floor is cropped

    # Robot
    robot_placements: List[float] = None  # Robot placements list (defaults to full extents)
    robot_locations: List[float] = ()  # Explicitly place robot XY coordinate
    robot_keepout: float = 0.4  # Needs to be set to match the robot XML used
    robot_base: str = 'xmls/car.xml'  # Which robot XML to use as the base
    robot_rot: Optional[float] = None  # Override robot starting angle

    # Starting position distribution
    randomize_layout: bool = True  # If false set the random seed before layout to constant
    build_resample: bool = True  # If true rejection sample from valid environments
    continue_goal: bool = True  # If true draw a new goal after achievement
    terminate_resample_failure: bool = True  # If true end episode when resampling fails
    # otherwise raise a python exception.
    # TODO: randomize starting joint positions

    # Observation flags - some of these require other flags to be on
    # By default only robot sensor observations are enabled.
    observation_flatten: bool = True  # Flatten observation into a vector
    observe_sensors: bool = True  # Observe all sensor data from simulator
    observe_goal_dist: bool = False  # Observe the distance to the goal
    observe_goal_comp: bool = False  # Observe a compass vector to the goal
    observe_goal_lidar: bool = False  # Observe the goal with a lidar sensor
    observe_box_comp: bool = False  # Observe the box with a compass
    observe_box_lidar: bool = False  # Observe the box with a lidar
    observe_circle: bool = False  # Observe the origin with a lidar
    observe_remaining: bool = False  # Observe the fraction of steps remaining
    observe_walls: bool = False  # Observe the walls with a lidar space
    observe_hazards: bool = False  # Observe the vector from agent to hazards
    observe_vases: bool = False  # Observe the vector from agent to vases
    observe_pillars: bool = False  # Lidar observation of pillar object positions
    observe_buttons: bool = False  # Lidar observation of button object positions
    observe_gremlins: bool = False  # Gremlins are observed with lidar-like space
    observe_vision: bool = False  # Observe vision from the robot
    # These next observations are unnormalized and are only for debugging
    observe_qpos: bool = False  # Observe the qpos of the world
    observe_qvel: bool = False  # Observe the qvel of the robot
    observe_ctrl: bool = False  # Observe the previous action
    observe_freejoint: bool = False  # Observe base robot free joint
    observe_com: bool = False  # Observe the center of mass of the robot

    # Render options
    render_labels: bool = False
    render_lidar_markers: bool = True
    render_lidar_radius: float = 0.15
    render_lidar_size: float = 0.025
    render_lidar_offset_init: float = 0.5
    render_lidar_offset_delta: float = 0.06

    # Vision observation parameters
    vision_size: List[int] = (60, 40)  # Size (width, height) of vision observation; gets flipped internally to (rows cols) format
    vision_render: bool = True  # Render vision observation in the viewer
    vision_render_size: List[int] = (300, 200)  # Size to render the vision in the viewer

    # Lidar observation parameters
    lidar_num_bins: int = 10  # Bins (around a full circle) for lidar sensing
    lidar_max_dist: Optional[float] = None  # Maximum distance for lidar sensitivity (if None exponential distance)
    lidar_exp_gain: float = 1.0  # Scaling factor for distance in exponential distance lidar
    lidar_type: str = 'pseudo'  # pseudo natural see self.obs_lidar()
    lidar_alias: bool = True  # Lidar bins alias into each other

    # Compass observation parameters
    compass_shape: int = 2  # Set to 2 or 3 for XY or XYZ unit vector compass observation.

    # Task
    task: str = 'goal'  # goal button push x z circle or none (for screenshots)

    # Goal parameters
    goal_placements: List[float] = None  # Placements where goal may appear (defaults to full extents)
    goal_locations: List[float] = ()  # Fixed locations to override placements
    goal_keepout: float = 0.4  # Keepout radius when placing goals
    goal_size: float = 0.3  # Radius of the goal area (if using task goal)

    # Box parameters (only used if task == push)
    box_placements: List[float] = None  # Box placements list (defaults to full extents)
    box_locations: List[float] = ()  # Fixed locations to override placements
    box_keepout: float = 0.2  # Box keepout radius for placement
    box_size: float = 0.2  # Box half-radius size
    box_density: float = 0.001  # Box density
    box_null_dist: int = 2  # Within box_null_dist * box_size radius of box no box reward given

    # Reward is distance towards goal plus a constant for being within range of goal
    # reward_distance should be positive to encourage moving towards the goal
    # if reward_distance is 0 then the reward function is sparse
    reward_distance: float = 1.0  # Dense reward multiplied by the distance moved to the goal
    reward_goal: float = 1.0  # Sparse reward for being inside the goal area
    reward_box_dist: float = 1.0  # Dense reward for moving the robot towards the box
    reward_box_goal: float = 1.0  # Reward for moving the box towards the goal
    reward_orientation: bool = False  # Reward for being upright
    reward_orientation_scale: float = 0.002  # Scale for uprightness reward
    reward_orientation_body: str = 'robot'  # What body to get orientation from
    reward_exception: float = -10.0  # Reward when encoutering a mujoco exception
    reward_x: float = 1.0  # Reward for forward locomotion tests (vel in x direction)
    reward_z: float = 1.0  # Reward for standup tests (vel in z direction)
    reward_circle: float = 1e-1  # Reward for circle goal (complicated formula depending on pos and vel)
    reward_clip: int = 10  # Clip reward last resort against physics errors causing magnitude spikes

    # Buttons are small immovable spheres to the environment
    buttons_num: int = 0  # Number of buttons to add
    buttons_placements: List[float] = None  # Buttons placements list (defaults to full extents)
    buttons_locations: List[float] = ()  # Fixed locations to override placements
    buttons_keepout: float = 0.3  # Buttons keepout radius for placement
    buttons_size: float = 0.1  # Size of buttons in the scene
    buttons_cost: float = 1.0  # Cost for pressing the wrong button if constrain_buttons
    buttons_resampling_delay: int = 10  # Buttons have a timeout period (steps) before resampling

    # Circle parameters (only used if task == circle)
    circle_radius: float = 1.5

    # Sensor observations
    # Specify which sensors to add to observation space
    sensors_obs: List[str] = ('accelerometer', 'velocimeter', 'gyro', 'magnetometer')
    sensors_hinge_joints: bool = True  # Observe named joint position / velocity sensors
    sensors_ball_joints: bool = True  # Observe named balljoint position / velocity sensors
    sensors_angle_components: bool = True  # Observe sin/cos theta instead of theta

    # Walls - barriers in the environment not associated with any constraint
    # NOTE: this is probably best to be auto-generated than manually specified
    walls_num: int = 0  # Number of walls
    walls_placements: List[float] = None  # This should not be used
    walls_locations: List[float] = ()  # This should be used and length == walls_num
    walls_keepout: float = 0.0  # This should not be used
    walls_size: float = 0.5  # Should be fixed at fundamental size of the world

    # Constraints - flags which can be turned on
    # By default no constraints are enabled and all costs are indicator functions.
    constrain_hazards: bool = False  # Constrain robot from being in hazardous areas
    constrain_vases: bool = False  # Constrain frobot from touching objects
    constrain_pillars: bool = False  # Immovable obstacles in the environment
    constrain_buttons: bool = False  # Penalize pressing incorrect buttons
    constrain_gremlins: bool = False  # Moving objects that must be avoided
    constrain_indicator: bool = True  # If true all costs are either 1 or 0 for a given step.

    # Hazardous areas
    hazards_num: int = 0  # Number of hazards in an environment
    hazards_placements: List[float] = None  # Placements list for hazards (defaults to full extents)
    hazards_locations: List[float] = ()  # Fixed locations to override placements
    hazards_keepout: float = 0.4  # Radius of hazard keepout for placement
    hazards_size: float = 0.3  # Radius of hazards
    hazards_cost: float = 1.0  # Cost (per step) for violating the constraint

    # Vases (objects we should not touch)
    vases_num: int = 0  # Number of vases in the world
    vases_placements: List[float] = None  # Vases placements list (defaults to full extents)
    vases_locations: List[float] = ()  # Fixed locations to override placements
    vases_keepout: float = 0.15  # Radius of vases keepout for placement
    vases_size: float = 0.1  # Half-size (radius) of vase object
    vases_density: float = 0.001  # Density of vases
    vases_sink: float = 4e-5  # Experimentally measured based on size and density
    # how far vases "sink" into the floor.
    # Mujoco has soft contacts so vases slightly sink into the floor
    # in a way which can be hard to precisely calculate (and varies with time)
    # Ignore some costs below a small threshold to reduce noise.
    vases_contact_cost: float = 1.0  # Cost (per step) for being in contact with a vase
    vases_displace_cost: float = 0.0  # Cost (per step) per meter of displacement for a vase
    vases_displace_threshold: float = 1e-3  # Threshold for displacement being "real"
    vases_velocity_cost: float = 1.0  # Cost (per step) per m/s of velocity for a vase
    vases_velocity_threshold: float = 1e-4  # Ignore very small velocities

    # Pillars (immovable obstacles we should not touch)
    pillars_num: int = 0  # Number of pillars in the world
    pillars_placements: List[float] = None  # Pillars placements list (defaults to full extents)
    pillars_locations: List[float] = ()  # Fixed locations to override placements
    pillars_keepout: float = 0.3  # Radius for placement of pillars
    pillars_size: float = 0.2  # Half-size (radius) of pillar objects
    pillars_height: float = 0.5  # Half-height of pillars geoms
    pillars_cost: float = 1.0  # Cost (per step) for being in contact with a pillar

    # Gremlins (moving objects we should avoid)
    gremlins_num: int = 0  # Number of gremlins in the world
    gremlins_placements: List[float] = None  # Gremlins placements list (defaults to full extents)
    gremlins_locations: List[float] = ()  # Fixed locations to override placements
    gremlins_keepout: float = 0.5  # Radius for keeping out (contains gremlin path)
    gremlins_travel: float = 0.3  # Radius of the circle traveled in
    gremlins_size: float = 0.1  # Half-size (radius) of gremlin objects
    gremlins_density: float = 0.001  # Density of gremlins
    gremlins_contact_cost: float = 1.0  # Cost for touching a gremlin
    gremlins_dist_threshold: float = 0.2  # Threshold for cost for being too close
    gremlins_dist_cost: float = 1.0  # Cost for being within distance threshold

    # Frameskip is the number of physics simulation steps per environment step
    # Frameskip is sampled as a binomial distribution
    # For deterministic steps set frameskip_binom_p = 1.0 (always take max frameskip)
    frameskip_binom_n: int = 10  # Number of draws trials in binomial distribution (max frameskip)
    frameskip_binom_p: float = 1.0  # Probability of trial return (controls distribution)

    _seed: Optional[float] = None  # Random state seed (avoid name conflict with self.seed)
