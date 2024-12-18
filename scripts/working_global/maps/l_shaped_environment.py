from mpscenes.obstacles.box_obstacle import BoxObstacle
from mpscenes.obstacles.dynamic_cylinder_obstacle import DynamicCylinderObstacle
import os
import numpy as np


class LShapedEnvironment:
    def __init__(self, first_part_lenght, second_part_lenght, width, wall_height=0.8, wall_thickness=0.1, elevation=0.0, scaling=1.0):
        # lenght and width changed around
        self.first_part_lenght = first_part_lenght
        self.second_part_lenght = second_part_lenght
        self.width = width
        self.wall_height = wall_height
        self.wall_thickness =  wall_thickness
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
                'type': 'box', # long wall 2
                'geometry': {
                    'position': [-(self.second_part_lenght*0.5 - self.width), -self.width, self.z_position],
                    'width':  self.wall_thickness, # Cover the entire length
                    'height': self.wall_height,
                    'length': self.second_part_lenght
                }},
            {
                'type': 'box', # long wall 1 
                'geometry': {
                    'position': [self.width, self.first_part_lenght*0.5 - self.width, self.z_position],
                    'width': self.first_part_lenght,  # Cover the entire length
                    'height': self.wall_height,
                    'length': self.wall_thickness,
                }},
            {
                'type': 'box',
                'geometry': {
                    'position': [-(self.second_part_lenght-self.width)*0.5, 0, self.z_position],
                    'width': self.wall_thickness,  # Cover the entire length
                    'height': self.wall_height,
                    'length': (self.second_part_lenght-self.width),
                }},
            {
                'type': 'box',
                'geometry': {
                    'position': [0, (self.first_part_lenght-self.width)*0.5, self.z_position],
                    'width': (self.first_part_lenght-self.width),  # Cover the entire length
                    'height': self.wall_height,
                    'length': self.wall_thickness,
                }},
            ]
    
        # add walls to obstacles list
        for i, obst_dict in enumerate(wall_obstacles_dicts):
            box_obstacle = BoxObstacle(name=f"wall_{i}", content_dict=obst_dict)
            self.obstacles.append(box_obstacle)
        print("Generated walls")

    def generate_static_obstacle_1_left(self, position_offset=15, width_scaling=1.0, length_scaling=1.0):
        # Adjust obstacle dimensions using the scaling parameter
        scaled_length = self.obstacle_1_length * width_scaling 
        scaled_width = self.obstacle_1_width * length_scaling
        scaled_height = self.obstacle_1_height

        # Adjust position using the horizontal offset
        obstacle_1_position = [(self.width - scaled_width*0.5),
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
            }}
        
        # make obstacle and add to list
        static_obstacle_1 = BoxObstacle(name="static_box_1", content_dict=obstacle_1)
        self.obstacles.append(static_obstacle_1)
        
        print('Generated Static Obstacle 1')
        

    def generate_static_obstacle_1_right(self, position_offset=5, width_scaling=1.0, length_scaling=1.0):
        # Adjust obstacle dimensions using the scaling parameter
        scaled_length = self.obstacle_1_length * width_scaling 
        scaled_width = self.obstacle_1_width * length_scaling
        scaled_height = self.obstacle_1_height

        # Adjust position using the horizontal offset
        obstacle_1_position = [(scaled_width*0.5),
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
            }}
        

        # make obstacle and add to list
        static_obstacle_1 = BoxObstacle(name="static_box_2", content_dict=obstacle_1)
        self.obstacles.append(static_obstacle_1)
        
        print('Generated Static Obstacle 2')

    def generate_static_obstacle_2_right(self, position_offset=-5, width_scaling=1.0, length_scaling=1.0):
        # Adjust obstacle dimensions using the scaling parameter
        scaled_length = self.obstacle_1_length * width_scaling 
        scaled_width = self.obstacle_1_width * length_scaling
        scaled_height = self.obstacle_1_height

        # Adjust position using the horizontal offset
        obstacle_1_position = [position_offset,
                               -(scaled_width*0.5),# Centered along the width
                                scaled_height / 2]

        # Define the obstacle
        obstacle_1 = {
            'type': 'box',
            'geometry': {
                'position': obstacle_1_position,
                'width': scaled_width,
                'height': scaled_height,
                'length': scaled_length
            }}
        

        # make obstacle and add to list
        static_obstacle_1 = BoxObstacle(name="static_box_4", content_dict=obstacle_1)
        self.obstacles.append(static_obstacle_1)
        
        print('Generated Static Obstacle 4')

    def generate_static_obstacle_2_left(self, position_offset=-5, width_scaling=1.0, length_scaling=1.0):
        # Adjust obstacle dimensions using the scaling parameter
        scaled_length = self.obstacle_1_length * width_scaling 
        scaled_width = self.obstacle_1_width * length_scaling
        scaled_height = self.obstacle_1_height

        # Adjust position using the horizontal offset
        obstacle_1_position = [position_offset,
                            -(self.width - scaled_width*0.5),# Centered along the width
                                scaled_height / 2]

        # Define the obstacle
        obstacle_1 = {
            'type': 'box',
            'geometry': {
                'position': obstacle_1_position,
                'width': scaled_width,
                'height': scaled_height,
                'length': scaled_length
            }}
        

        # make obstacle and add to list
        static_obstacle_1 = BoxObstacle(name="static_box_3", content_dict=obstacle_1)
        self.obstacles.append(static_obstacle_1)
        
        print('Generated Static Obstacle 3')

    def generate_dynamic_obstacle_1(self, position_offset=15, radius=0.5, height=1, frequency=3, speed_scaling=3):
        
        h = height/2

        base_duration = 10
        duration = (base_duration * frequency) / speed_scaling

        # Define the left and right positions
        left_position = [radius, position_offset, h]
        right_position = [ self.width - radius, position_offset, h]

        control_points = []

        for i in range(frequency): 
            if i % 2 == 0:
                control_points.append(left_position)  # Add left point
            else:
                control_points.append(right_position)  # Add right point

        if frequency % 2 == 0:
            control_points.append(left_position)
        else:
            control_points.append(right_position)



        splineDict = {"degree": 1,
                    "controlPoints": control_points,
                    "duration": duration}
        
        config_dict={
            "type": "cylinder",
            "geometry": {
                "trajectory": splineDict,
                "radius": radius,
                "height": height},
            "movable": False,
            "rgba": [1.0, 0.0, 0.0, 1.0]}
        
        dynamic_cylinder = DynamicCylinderObstacle(name="dynamic_cylinder_2",  content_dict=config_dict)

        self.obstacles.append(dynamic_cylinder)


    def generate_dynamic_obstacle_2(self, position_offset=-10, radius=0.5, height=1, frequency=3, speed_scaling=3):
        
        h = height/2

        base_duration = 10
        duration = (base_duration * frequency) / speed_scaling

        # Define the left and right positions
        left_position = [position_offset, -(self.width- radius),h]
        right_position = [position_offset,- radius, h]

        control_points = []

        for i in range(frequency): 
            if i % 2 == 0:
                control_points.append(left_position)  # Add left point
            else:
                control_points.append(right_position)  # Add right point

        if frequency % 2 == 0:
            control_points.append(left_position)
        else:
            control_points.append(right_position)



        splineDict = {"degree": 1,
                    "controlPoints": control_points,
                    "duration": duration}
        
        config_dict={
            "type": "cylinder",
            "geometry": {
                "trajectory": splineDict,
                "radius": radius,
                "height": height},
            "movable": False,
            "rgba": [1.0, 0.0, 0.0, 1.0]}
        
        dynamic_cylinder = DynamicCylinderObstacle(name="dynamic_cylinder_2",  content_dict=config_dict)

        self.obstacles.append(dynamic_cylinder)

    def get_obstacles(self):
        return self.obstacles