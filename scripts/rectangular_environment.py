from mpscenes.obstacles.box_obstacle import BoxObstacle

class RectangularEnvironment:
    def __init__(self, length, width, wall_height=0.8, wall_thickness=0.1, elevation=0.0, scaling=1.0):
        # lenght and width changed around
        self.length = width * scaling
        self.width = length * scaling
        self.wall_height = wall_height
        self.wall_thickness = wall_thickness
        self.elevation = elevation
        self.z_position = (wall_height / 2) + elevation
        self.obstacles = []

        # semi-private, dimension values for obstacles
        self.obstacle_1_length = 4
        self.obstacle_1_width =  4
        self.obstacle_1_height = self.wall_height
        self.obstacle_2_length = self.obstacle_1_length
        self.obstacle_2_width =  self.obstacle_1_width
        self.obstacle_2_height = self.wall_height
    
    def generate_walls(self):
        wall_obstacles_dicts = [
            # Bottom wall
            {
                'type': 'box',
                'geometry': {
                    'position': [0.0, -(self.width / 2.0), self.z_position],
                    'width': self.wall_thickness,  # Cover the entire length
                    'height': self.wall_height,
                    'length': self.length
                },
                'high': {
                    'position': [0.0, -(self.width / 2.0), self.z_position],
                    'width': self.wall_thickness,  # Cover the entire length
                    'height': self.wall_height,
                    'length': self.length
                },
                'low': {
                    'position': [0.0, -(self.width / 2.0), self.z_position],
                    'width': self.wall_thickness,  # Cover the entire length
                    'height': self.wall_height,
                    'length': self.length
                },
            },
            {
                'type': 'box',
                'geometry': {
                    'position': [0.0, (self.width / 2.0), self.z_position],
                    'width': self.wall_thickness,  # Cover the entire length
                    'height': self.wall_height,
                    'length': self.length
                },
                'high': {
                    'position': [0.0, (self.width / 2.0), self.z_position],
                    'width': self.wall_thickness,  # Cover the entire length
                    'height': self.wall_height,
                    'length': self.length
                },
                'low': {
                    'position': [0.0, (self.width / 2.0), self.z_position],
                    'width': self.wall_thickness,  # Cover the entire length
                    'height': self.wall_height,
                    'length': self.length
                },
            },
            
            {
                'type': 'box',
                'geometry': {
                    'position': [self.length/2, 0.0, self.z_position],
                    'width': self.width,  # Cover the entire length
                    'height': self.wall_height,
                    'length': self.wall_thickness
                },
                'high': {
                    'position': [self.length/2, 0.0, self.z_position],
                    'width': self.width,  # Cover the entire length
                    'height': self.wall_height,
                    'length': self.wall_thickness
                },
                'low': {
                    'position': [self.length/2, 0.0, self.z_position],
                    'width': self.width,  # Cover the entire length
                    'height': self.wall_height,
                    'length': self.wall_thickness
                },
            },
            {
                'type': 'box',
                'geometry': {
                    'position': [-self.length/2, 0.0, self.z_position],
                    'width': self.width,  # Cover the entire length
                    'height': self.wall_height,
                    'length': self.wall_thickness
                },
                'high': {
                    'position': [-self.length/2, 0.0, self.z_position],
                    'width': self.width,  # Cover the entire length
                    'height': self.wall_height,
                    'length': self.wall_thickness
                },
                'low': {
                    'position': [-self.length/2, 0.0, self.z_position],
                    'width': self.width,  # Cover the entire length
                    'height': self.wall_height,
                    'length': self.wall_thickness
                }
            }
            ]
        
        # add walls to obstacles list
        for i, obst_dict in enumerate(wall_obstacles_dicts):
            box_obstacle = BoxObstacle(name=f"wall_{i}", content_dict=obst_dict)
            self.obstacles.append(box_obstacle)
        print("Generated walls")


    def generate_static_obstacle_1(self, position_offset=-15, width_scaling=1.0, length_scaling=1.0):
        # Adjust obstacle dimensions using the scaling parameter
        scaled_length = self.obstacle_1_length * width_scaling 
        scaled_width = self.obstacle_1_width * length_scaling
        scaled_height = self.obstacle_1_height

        # Adjust position using the horizontal offset
        obstacle_1_position = [
            (self.length / 2) - (scaled_length / 2),
            position_offset,  # Centered along the width
            scaled_height / 2]

        # Define the obstacle
        obstacle_1 = {
            'type': 'box',
            'geometry': {
                'position': obstacle_1_position,
                'width': scaled_width,
                'height': scaled_height,
                'length': scaled_length
            },
            'high': {
                'position': obstacle_1_position,
                'width': scaled_width,
                'height': scaled_height,
                'length': scaled_length
            },
            'low': {
                'position': obstacle_1_position,
                'width': scaled_width,
                'height': scaled_height,
                'length': scaled_length}}
        

        # make obstacle and add to list
        static_obstacle_1 = BoxObstacle(name="static_box_1", content_dict=obstacle_1)
        self.obstacles.append(static_obstacle_1)
        
        print('Generated Static Obstacle 2')

    def generate_static_obstacle_2(self, position_offset= 5, width_scaling=2.0, length_scaling=1.0):
        # Adjust obstacle dimensions using the scaling parameter
        scaled_length = self.obstacle_1_length * width_scaling
        scaled_width = self.obstacle_1_width * length_scaling
        scaled_height = self.obstacle_1_height

        # Adjust position using the horizontal offset
        obstacle_position = [
            - (self.length / 2) + (scaled_length / 2),
            position_offset,  # Centered along the width
            scaled_height / 2]

        # Define the obstacle
        obstacle_1 = {
            'type': 'box',
            'geometry': {
                'position': obstacle_position,
                'width': scaled_width,
                'height': scaled_height,
                'length': scaled_length
            },
            'high': {
                'position': obstacle_position,
                'width': scaled_width,
                'height': scaled_height,
                'length': scaled_length
            },
            'low': {
                'position': obstacle_position,
                'width': scaled_width,
                'height': scaled_height,
                'length': scaled_length}}
        

        # make obstacle and add to list
        static_obstacle_1 = BoxObstacle(name="static_box_1", content_dict=obstacle_1)
        self.obstacles.append(static_obstacle_1)
        
        print('Generated Static Obstacle 2')

    def get_obstacles(self):
        return self.obstacles