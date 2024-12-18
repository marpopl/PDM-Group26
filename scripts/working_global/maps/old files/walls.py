from mpscenes.obstacles.box_obstacle import BoxObstacle

def generate_wall_obstacles(length, width, wall_height=0.8, wall_thickness=0.1, elevation=0.0):
    z_position = (wall_height / 2) + elevation
    
    wall_obstacles_dicts = [
        # Bottom wall
        {
            'type': 'box',
            'geometry': {
                'position': [0.0, -(width / 2.0), z_position],
                'width': wall_thickness,  # Cover the entire length
                'height': wall_height,
                'length': length
            },
            'high': {
                'position': [0.0, -(width / 2.0), z_position],
                'width': wall_thickness,  # Cover the entire length
                'height': wall_height,
                'length': length
            },
            'low': {
                'position': [0.0, -(width / 2.0), z_position],
                'width': wall_thickness,  # Cover the entire length
                'height': wall_height,
                'length': length
            },
        },
        {
            'type': 'box',
            'geometry': {
                'position': [0.0, (width / 2.0), z_position],
                'width': wall_thickness,  # Cover the entire length
                'height': wall_height,
                'length': length
            },
            'high': {
                'position': [0.0, (width / 2.0), z_position],
                'width': wall_thickness,  # Cover the entire length
                'height': wall_height,
                'length': length
            },
            'low': {
                'position': [0.0, (width / 2.0), z_position],
                'width': wall_thickness,  # Cover the entire length
                'height': wall_height,
                'length': length
            },
        },
        
        {
            'type': 'box',
            'geometry': {
                'position': [length/2, 0.0, z_position],
                'width': width,  # Cover the entire length
                'height': wall_height,
                'length': wall_thickness
            },
            'high': {
                'position': [length/2, 0.0, z_position],
                'width': width,  # Cover the entire length
                'height': wall_height,
                'length': wall_thickness
            },
            'low': {
                'position': [length/2, 0.0, z_position],
                'width': width,  # Cover the entire length
                'height': wall_height,
                'length': wall_thickness
            },
        },
        {
            'type': 'box',
            'geometry': {
                'position': [-length/2, 0.0, z_position],
                'width': width,  # Cover the entire length
                'height': wall_height,
                'length': wall_thickness
            },
            'high': {
                'position': [-length/2, 0.0, z_position],
                'width': width,  # Cover the entire length
                'height': wall_height,
                'length': wall_thickness
            },
            'low': {
                'position': [-length/2, 0.0, z_position],
                'width': width,  # Cover the entire length
                'height': wall_height,
                'length': wall_thickness
            },
        }
        ]
    
    walls = [BoxObstacle(name=f"wall_{i}", content_dict=obst_dict) for i, obst_dict in enumerate(wall_obstacles_dicts)]
        
    return walls
    
    
    
