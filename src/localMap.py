from ransac import LineModel
import math

class Wall:
  def __init__(self, line):
    self.lineModel = line
    self.polar = [line.distanceToPoint([0,0]), atan2(line.trajectory[1], line.trajectory[0])]

class LocalMap:
  def __init__(self):
    self.walls = []
    self.corners = []
    self.minAngleDiff = 5.0 * math.pi / 180.0
    self.minDistDiff = 0.4
  def wallIs(self, w):
    if not wallExists(w):
      walls.push(Wall(w))
      for wall in self.walls:
        isect = wall.intersection(w)
        if isect is not None:
          self.corners.push(wall.intersection(w))
  def wallExists(self, w):
    for wall in self.walls:
      if wall.angleBetween(w) < self.minAngleDiff and wall.distanceToPoint(w.origin) < self.minDistDiff:
        return True
    return False
