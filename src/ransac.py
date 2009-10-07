import random

# given some points, randomly chooses num_points_necessary_for_fit 500 or so
# times.  At each iteration model_callback is called taking as an argument a
# list of num_points_necessary_for_fit points with which it generates a black box
# model object.  when model is passed into inlierp_callback along with a
# candidate point, inlierp_callback will return true if the given model
# yielded an inlier when the point was transformed by the model.
# returns the following values in an array:
# 1. the best model.
# 2. the inliers on that model
def ransac(points, num_points_necessary_for_fit, model_callback, inlierp_callback):
    best_model = None
    best_inliers = []
    mutable_points = points[:]
    def random_partition():
        random.shuffle(mutable_points)
        return [ mutable_points[:num_points_necessary_for_fit], mutable_points[num_points_necessary_for_fit:]]
        
    for iteration in xrange(0, 150):
        [fit_points, test_points] = random_partition()
        model = model_callback(fit_points)
        inliers = filter(lambda test_point : inlierp_callback(model, test_point), test_points)
        if ((not best_model) or (len(inliers) > len(best_inliers))):
           best_model = model
           best_inliers = inliers
    
    return [best_model, best_inliers]

# RANSAC for fitting 2d/3d lines to a point cloud

def cross3d(v, w):
    x = v[1]*w[2] - v[2]*w[1]
    y = v[2]*w[0] - v[0]*w[2]
    z = v[0]*w[1] - v[1]*w[0]
    return (x, y, z)

def cross2d(v, w):
    return cross3d([v[0], v[1], 0], [w[0], w[1], 0])

def cross(v,w):
    if len(v) == 2:
        return cross2d(v,w)
    else:
        return cross3d(v,w)

def vector_minus(b, a):
    a_to_b = map(lambda acoord, bcoord: bcoord - acoord, a, b)
    return a_to_b

def vector_scale(v, s):
    return map(lambda x: x * s, v)

# v is a tuple representing a 3d vector
def vector_length(v):
    sum = 0
    for x in v:
        sum = sum + x*x
    return sum ** 0.5
    
def normalize(v):
    l = vector_length(v)
    return map(lambda x : x / l, v)

def vector_dot(v,w):
    return sum(map( lambda a,b: a * b, v, w))

class LineModel:
    def __init__(self, two_cartesian_points):
        [a, b] = two_cartesian_points
        a_to_b = map(lambda acoord, bcoord: bcoord - acoord, a, b)
        self.origin = a
        self.trajectory = normalize(a_to_b)
    def angleBetween(self, anotherLineModel):
        dot_result =  vector_dot(self.trajectory, anotherLineModel.trajectory)
        return math.acos(dot_result)
    def distanceToPoint(self, pt):
        w = vector_minus(pt, self.origin)
        traj_cross_w = cross(self.trajectory, w)
        return vector_length(traj_cross_w) / vector_length(self.trajectory)
        
def fitLineWithRansac(points, distance_cutoff):
    return ransac(points, 2, LineModel, lambda model, pt: model.distanceToPoint(pt) < distance_cutoff)
    
    
