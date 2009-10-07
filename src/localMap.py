from ransac import LineModel
import math

class Wall:
  def __init__(self, line):
    self.lineModel = line
    self.polar = [line.distanceToPoint([0,0]), math.atan2(line.trajectory[1], line.trajectory[0])]
  def angleBetween(self, line):
    return self.lineModel.angleBetween(line)
  def origin(self):
    return self.lineModel.origin
  def trajectory(self):
    return self.lineModel.trajectory

class LocalMap:
  def __init__(self):
    self.walls = []  # list of walls, each element is of the format [wall, confidence]
    self.maxAngleDiff = 5.0 * math.pi / 180.0  # the maximum degree difference to be the same wall
    self.maxDistDiff = 0.4  # the maximum distance apart to be the same wall

  def wallIs(self, line):
    i = self.wallIndex(line)  # returns an index if close enough, -1 otherwise
    if i < 0:  # no similar wall found
      if self.isRectilinear(line):  # this one will work
        self.walls.append([Wall(line), 1])
    else:  # similar wall was found
      self.walls[i][1] += 1  # add confidence value

  def wallIndex(self, line):
    for wall in self.walls:
      if wall[0].angleBetween(line) < self.maxAngleDiff and wall[0].distanceToPoint(line.origin) < self.maxDistDiff:
        return self.walls.index(wall)
    return -1

  def isRectilinear(self, line):
    if len(self.walls) == 0:
      return True
    angleTests = [0, math.pi/2.0, math.pi, 3.0*math.pi/2.0, 2*math.pi]
    for wall in self.walls:
      a = wall[0].angleBetween(line)
      for test in angleTests:
        if a > test-self.maxAngleDiff and a < test+self.maxAngleDiff:
          return True
    return False
            
