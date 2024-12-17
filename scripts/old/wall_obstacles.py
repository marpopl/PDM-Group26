from mpscenes.obstacles.box_obstacle import BoxObstacle

## Define customizable wall lengths and distances
wall_length_outer = 20
wall_length_inner = 15

# Double the distance between the inner and outer walls
gap_multiplier = 5
adjusted_wall_length_inner = wall_length_outer - (wall_length_outer - wall_length_inner) * gap_multiplier

# Wall obstacle definitions
wall_obstacles_dicts = [
    # Outer ring
    {
        'type': 'box', 
         'geometry': {
             'position': [wall_length_outer/2.0, 0.0, 0.4], 'width': wall_length_outer, 'height': 0.8, 'length': 0.1
        },
        'high': {
            'position' : [wall_length_outer/2.0, 0.0, 0.4],
            'width': wall_length_outer,
            'height': 0.8,
            'length': 0.1,
        },
        'low': {
            'position' : [wall_length_outer/2.0, 0.0, 0.4],
            'width': wall_length_outer,
            'height': 0.8,
            'length': 0.1,
        },
    },
    {
        'type': 'box', 
         'geometry': {
             'position': [0.0, wall_length_outer/2.0, 0.4], 'width': 0.1, 'height': 0.8, 'length': wall_length_outer
        },
        'high': {
            'position' : [0.0, wall_length_outer/2.0, 0.4],
            'width': 0.1,
            'height': 0.8,
            'length': wall_length_outer,
        },
        'low': {
            'position' : [0.0, wall_length_outer/2.0, 0.4],
            'width': 0.1,
            'height': 0.8,
            'length': wall_length_outer,
        },
    },
    {
        'type': 'box', 
         'geometry': {
             'position': [0.0, -wall_length_outer/2.0, 0.4], 'width': 0.1, 'height': 0.8, 'length': wall_length_outer
        },
        'high': {
            'position' : [0.0, -wall_length_outer/2.0, 0.4],
            'width': 0.1,
            'height': 0.8,
            'length': wall_length_outer,
        },
        'low': {
            'position' : [0.0, -wall_length_outer/2.0, 0.4],
            'width': 0.1,
            'height': 0.8,
            'length': wall_length_outer,
        },
    },
    {
        'type': 'box', 
         'geometry': {
             'position': [-wall_length_outer/2.0, 0.0, 0.4], 'width': wall_length_outer, 'height': 0.8, 'length': 0.1
        },
        'high': {
            'position' : [-wall_length_outer/2.0, 0.0, 0.4],
            'width': wall_length_outer,
            'height': 0.8,
            'length': 0.1,
        },
        'low': {
            'position' : [-wall_length_outer/2.0, 0.0, 0.4],
            'width': wall_length_outer,
            'height': 0.8,
            'length': 0.1,
        },
    },
    # Inner ring (adjusted)
    {
        'type': 'box', 
         'geometry': {
             'position': [adjusted_wall_length_inner/2.0, 0.0, 0.4], 'width': adjusted_wall_length_inner, 'height': 0.8, 'length': 0.1
        },
        'high': {
            'position' : [adjusted_wall_length_inner/2.0, 0.0, 0.4],
            'width': adjusted_wall_length_inner,
            'height': 0.8,
            'length': 0.1,
        },
        'low': {
            'position' : [adjusted_wall_length_inner/2.0, 0.0, 0.4],
            'width': adjusted_wall_length_inner,
            'height': 0.8,
            'length': 0.1,
        },
    },
    {
        'type': 'box', 
         'geometry': {
             'position': [0.0, adjusted_wall_length_inner/2.0, 0.4], 'width': 0.1, 'height': 0.8, 'length': adjusted_wall_length_inner
        },
        'high': {
            'position' : [0.0, adjusted_wall_length_inner/2.0, 0.4],
            'width': 0.1,
            'height': 0.8,
            'length': adjusted_wall_length_inner,
        },
        'low': {
            'position' : [0.0, adjusted_wall_length_inner/2.0, 0.4],
            'width': 0.1,
            'height': 0.8,
            'length': adjusted_wall_length_inner,
        },
    },
    {
        'type': 'box', 
         'geometry': {
             'position': [0.0, -adjusted_wall_length_inner/2.0, 0.4], 'width': 0.1, 'height': 0.8, 'length': adjusted_wall_length_inner
        },
        'high': {
            'position' : [0.0, -adjusted_wall_length_inner/2.0, 0.4],
            'width': 0.1,
            'height': 0.8,
            'length': adjusted_wall_length_inner,
        },
        'low': {
            'position' : [0.0, -adjusted_wall_length_inner/2.0, 0.4],
            'width': 0.1,
            'height': 0.8,
            'length': adjusted_wall_length_inner,
        },
    },
    {
        'type': 'box', 
         'geometry': {
             'position': [-adjusted_wall_length_inner/2.0, 0.0, 0.4], 'width': adjusted_wall_length_inner, 'height': 0.8, 'length': 0.1
        },
        'high': {
            'position' : [-adjusted_wall_length_inner/2.0, 0.0, 0.4],
            'width': adjusted_wall_length_inner,
            'height': 0.8,
            'length': 0.1,
        },
        'low': {
            'position' : [-adjusted_wall_length_inner/2.0, 0.0, 0.4],
            'width': adjusted_wall_length_inner,
            'height': 0.8,
            'length': 0.1,
        },
    },
]

# Create wall obstacles
wall_obstacles = [BoxObstacle(name=f"wall_{i}", content_dict=obst_dict) for i, obst_dict in enumerate(wall_obstacles_dicts)]
