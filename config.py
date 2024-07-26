import os, sys, glob
# import constants as c

#TODO: temp left the config like this, then come back to modify it as needed  


"""

def get_proj_root():
    config_path = os.path.abspath(__file__)
    src_dir = os.path.dirname(config_path)
    proj_root = os.path.dirname(src_dir)

    return proj_root


def set_carla_api_path():
    proj_root = get_proj_root()

    dist_path = os.path.join(proj_root, "carla/PythonAPI/carla/dist")
    glob_path = os.path.join(dist_path, "carla-*%d.%d-%s.egg" % (
        sys.version_info.major,
        sys.version_info.minor,
        "win-amd64" if os.name == "nt" else "linux-x86_64"
    ))

    try:
        api_path = glob.glob(glob_path)[0]
    except IndexError:
        print("Couldn't set Carla API path.")
        exit(-1)

    if api_path not in sys.path:
        sys.path.append(api_path)
        print(f"API: {api_path}")

"""
class Config:
    """
    A class defining fuzzing configuration and helper methods.
    An instance of this class should be created by the main module (fuzzer.py)
    and then be shared across other modules as a context handler.
    """

    def __init__(self):
        # self.debug = False

        # simulator config
        # self.sim_host = "localhost"
        # self.sim_port = 2000
        # self.sim_tm_port = 8000

        # Fuzzer config
        self.max_cycles = 100 #TODO: temp as 100 
        # self.init_speed = 20
        # self.new_speed = 2 
        # self.change_distance=10
        # self.init_direction_x= 0
        # self.init_direction_y= 1
        # self.new_direction_x= 1
        # self.new_direction_y= -0.05
        #How to determine the changing scope?
        self.agent =  "Maneuver" #Maneuver; "Basic", "Behavior"
        self.behavior="normal"
        self.sync = True
        self.manual = False
        
        
        # self.max_mutation = 0
        # self.num_dry_runs = 1
        # self.num_param_mutations = 1
        # self.initial_quota = 10

        # Fuzzing metadata
        # self.cur_time = None
        # self.determ_seed = None
        self.temp_out_dir = "temp_data/" 
        #self.temp_out_dir = "temp_data/" 
        
        
        #TODO: save log file + scenario xosc file + video + parameter file(reproduce)
        
        # self.final_out_dir= "/media/w/New\ Volume/collision_data" #TODO: 需要根据不同的场景再进入子文件夹
        self.final_out_dir="final_data/" 
        #self.final_out_dir="final_data/" 
        
        #TODO: add the video saving part in our code as in the drivefuzz
        # self.seed_dir = None #TODO: maybe add it later 

        # Target config
        #basic, behavior or manually (no autoware)
        # self.agent_type = c.AUTOWARE # c.ATOWARE

        # Enable/disable Various Checks
        # self.check_dict = {
        #     "speed": True,
        #     "lane": False,
        #     "crash": True,
        #     "stuck": True,
        #     "red": False,
        #     "other": True,
        # }
        #what is this?
        # Functional testing
        # self.function = "general"

        # Sim-debug settings
        # self.view = c.BIRDSEYE
        """
        * Weather # XXX: really has to be float?
        # - float cloudiness: 0 (clear), 100 (cloudy)
        - float precipitation: 0 (none), 100 (heaviest rain)
        - float puddles: 0 (no puddle), 100 (completely covered with puddle)
        - float wind_intensity: 0 (no wind), 100 (strongest wind)
        - float fog_density: 0 (no fog), 100 (densest fog)
        # - float fog_distance: 0 (starts in the beginning), +inf (does not start)
        - float wetness: 0 (completely dry), 100 (completely wet)
        - float sun_azimuth_angle: 0 ~ 360
        - float sun_altitude_angle: -90 (midnight) ~ +90 (noon)
        """
        #here is the default value of the weather
        self.weather = {
        "cloud": 0,
        "rain": 0,
        "puddle": 0,
        "wind": 0,
        "fog": 0,
        "wetness": 0,
        "angle": 0,
        "altitude": 0
    }
            
            
    #init speed
    #init direction
    #change speed
    #change direction
    #change distance
    #different vehicle type cyclist, truck, vehicle 
    #weather -- 但是weather 在什么时候更改呢
    

    # def set_paths(self):
    #     self.queue_dir = os.path.join(self.out_dir, "queue")
    #     self.error_dir = os.path.join(self.out_dir, "errors")
    #     self.cov_dir = os.path.join(self.out_dir, "cov")
    #     self.meta_file = os.path.join(self.out_dir, "meta")
    #     self.cam_dir = os.path.join(self.out_dir, "camera")
    #     self.rosbag_dir = os.path.join(self.out_dir, "rosbags")
    #     self.score_dir = os.path.join(self.out_dir, "scores")

    # def enqueue_seed_scenarios(self):
    #     try:
    #         seed_scenarios = os.listdir(self.seed_dir)
    #     except:
    #         print("[-] Error - cannot find seed directory ({})".format(self.seed_dir))
    #         sys.exit(-1)

    #     queue = [seed for seed in seed_scenarios if not seed.startswith(".")
    #             and seed.endswith(".json")]

    #     return queue

