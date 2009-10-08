from line import LineModel

class Compass:
  def __init__(self):
    self.master = None
    self.history = [0]
    self.initialized = False
  def setMaster(self, line):
    self.master = line
    self.initialized = True
  def getOrientation(self, line):
    if self.master == None:
      self.setMaster(line)
      return 0
    return abs(line.angleBetween(self.master))
