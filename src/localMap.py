from ransac import LineModel
import math

class Wall:
  def __init__(self, line):
    self.lineModel = line
    self.polar = [line.distanceToPoint([0,0]), atan2(line.trajectory[1], line.trajectory[0])]

class LocalMap:
  def __init__(self):
    self.walls = []  # list of walls, each element is of the format [wall, confidence]
    self.maxAngleDiff = 5.0 * math.pi / 180.0  # the maximum degree difference to be the same wall
    self.maxDistDiff = 0.4  # the maximum distance apart to be the same wall
    self.rectilinearMin = math.pi / 2.0 - self.MaxAngleDiff  # minimum angle to be rectilinear
    self.rectilinearMax = math.pi / 2.0 + self.MaxAngleDiff

  def wallIs(self, w):
    i = self.wallIndex(w)  # returns an index if close enough, -1 otherwise
    if i < 0:  # no similar wall found
      if self.isRectilinear(w):  # this one will work
        walls.push([Wall(w), 1])
    else:  # similar wall was found
      walls[i][1] += 1  # add confidence value

  def wallIndex(self, w):
    for wall in self.walls:
      if wall.angleBetween(w) < self.maxAngleDiff and wall.distanceToPoint(w.origin) < self.maxDistDiff:
        return self.walls.index(wall)
    return -1

  def isRectilinear(self, w):
    for wall in self.walls:
      a = wall.angleBetween(w)
      if a > self.rectilinearMin and a < self.rectilinearMax:
        return True
    return False
            
