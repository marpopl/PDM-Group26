from mpscenes.obstacles.box_obstacle import BoxObstacle

# Define customizable wall lengths
wall_length_outer = 20
wall_length_inner = 15

# Wall 1: Position [10.0, 0.0]
# Wall 2: Position [0.0, 10.0]
# Wall 3: Position [-10.0, 0.0]
# Wall 4: Position [0.0, -10.0]
# Wall 5: Position [7.5, 0.0]
# Wall 6: Position [0.0, 7.5]
# Wall 7: Position [-7.5, 0.0]
# Wall 8: Position [0.0, -7.5]

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
    # Inner ring
        {
        'type': 'box', 
         'geometry': {
             'position': [wall_length_inner/2.0, 0.0, 0.4], 'width': wall_length_inner, 'height': 0.8, 'length': 0.1
        },
        'high': {
            'position' : [wall_length_inner/2.0, 0.0, 0.4],
            'width': wall_length_inner,
            'height': 0.8,
            'length': 0.1,
        },
        'low': {
            'position' : [wall_length_inner/2.0, 0.0, 0.4],
            'width': wall_length_inner,
            'height': 0.8,
            'length': 0.1,
        },
    },
    {
        'type': 'box', 
         'geometry': {
             'position': [0.0, wall_length_inner/2.0, 0.4], 'width': 0.1, 'height': 0.8, 'length': wall_length_inner
        },
        'high': {
            'position' : [0.0, wall_length_inner/2.0, 0.4],
            'width': 0.1,
            'height': 0.8,
            'length': wall_length_inner,
        },
        'low': {
            'position' : [0.0, wall_length_inner/2.0, 0.4],
            'width': 0.1,
            'height': 0.8,
            'length': wall_length_inner,
        },
    },
    {
        'type': 'box', 
         'geometry': {
             'position': [0.0, -wall_length_inner/2.0, 0.4], 'width': 0.1, 'height': 0.8, 'length': wall_length_inner
        },
        'high': {
            'position' : [0.0, -wall_length_inner/2.0, 0.4],
            'width': 0.1,
            'height': 0.8,
            'length': wall_length_inner,
        },
        'low': {
            'position' : [0.0, -wall_length_inner/2.0, 0.4],
            'width': 0.1,
            'height': 0.8,
            'length': wall_length_inner,
        },
    },
    {
        'type': 'box', 
         'geometry': {
             'position': [-wall_length_inner/2.0, 0.0, 0.4], 'width': wall_length_inner, 'height': 0.8, 'length': 0.1
        },
        'high': {
            'position' : [-wall_length_inner/2.0, 0.0, 0.4],
            'width': wall_length_inner,
            'height': 0.8,
            'length': 0.1,
        },
        'low': {
            'position' : [-wall_length_inner/2.0, 0.0, 0.4],
            'width': wall_length_inner,
            'height': 0.8,
            'length': 0.1,
        },
    },
]

wall_obstacles = [BoxObstacle(name=f"wall_{i}", content_dict=obst_dict) for i, obst_dict in enumerate(wall_obstacles_dicts)]
