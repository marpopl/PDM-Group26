from mpscenes.obstacles.box_obstacle import BoxObstacle

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