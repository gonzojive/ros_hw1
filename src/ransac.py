import random
import math

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
    
    left = right = best_inliers[0][0]
    bottom = top = best_inliers[0][1]
    for pt in best_inliers:
      if pt[0] < left:
        left = pt[0]
      if pt[0] > right:
        right = pt[0]
      if pt[1] < bottom:
        bottom = pt[1]
      if pt[1] > top:
        top = pt[1]
    return [best_model, best_inliers, [left, right, bottom, top]]

# RANSAC for fitting 2d/3d lines to a point cloud

