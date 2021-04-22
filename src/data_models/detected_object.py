class DetectedObject:
    x_middle = None
    y_middle = None
    z_middle = None
    obj_class = None
    obj_score = None

    def __init__(self, x_middle, y_middle, z_middle, obj_class, obj_score):
        self.x_middle = x_middle
        self.y_middle = y_middle
        self.z_middle = z_middle
        self.obj_class = obj_class
        self.obj_score = obj_score
