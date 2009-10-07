from ransac import LineModel
import math

class Wall:
  def __init__(self, line, extremes):
    self.lineModel = line
    self.polar = [line.distanceToPoint([0,0]), math.atan2(line.trajectory[1], line.trajectory[0])]
    self.leftmost = extremes[0]
    self.rightmost = extremes[1]
    self.bottommost = extremes[2]
    self.topmost = extremes[3]
    self.confidence = 1
  def angleBetween(self, wall):
    return self.lineModel.angleBetween(wall.lineModel)
  def distanceToPoint(self, point):
    return self.lineModel.distanceToPoint(point)
  def origin(self):
    return self.lineModel.origin
  def trajectory(self):
    return self.lineModel.trajectory

class LocalMap:
  def __init__(self):
    self.walls = []  # list of walls, each element is of the format [wall, confidence]
    self.maxAngleDiff = 5.0 * math.pi / 180.0  # the maximum degree difference to be the same wall
    self.maxDistDiff = 0.4  # the maximum distance apart to be the same wall

  def wallIs(self, line, extremes):
    newWall = Wall(line, extremes)
    i = self.wallIndex(newWall)  # returns an index if close enough, -1 otherwise
    if i < 0:  # no similar wall found
      if self.isRectilinear(newWall):  # this one will work
        self.walls.append(newWall)
    else:  # similar wall was found
      self.walls[i].confidence += 1  # add confidence value

  def wallIndex(self, w):
    for wall in self.walls:
      angle = wall.angleBetween(w)
      if (angle < self.maxAngleDiff or math.pi - angle < self.maxAngleDiff) and wall.distanceToPoint(w.origin()) < self.maxDistDiff:
        # check for any separation between the new wall and the old one
        if not (w.leftmost > wall.rightmost or w.rightmost < wall.leftmost or w.bottommost > wall.topmost or w.topmost < wall.bottommost):
          if w.leftmost < wall.leftmost:
            wall.leftmost = w.lefmost
          if w.rightmost > wall.rightmost:
            wall.rightmost = w.rightmost
          if w.bottommost < wall.bottommost:
            wall.bottommost = w.bottommost
          if w.topmost > wall.topmost:
            wall.topmost = w.topmost
          return self.walls.index(wall)
    return -1

  def isRectilinear(self, w):
    if len(self.walls) == 0:
      return True
    angleTests = [0, math.pi/2.0, math.pi, 3.0*math.pi/2.0, 2*math.pi]
    for wall in self.walls:
      a = wall.angleBetween(w)
      for test in angleTests:
        if a > test-self.maxAngleDiff and a < test+self.maxAngleDiff:
          return True
    return False
            
