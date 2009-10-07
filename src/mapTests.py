from localMap import *
import pdb

lm = LocalMap()

def createLine(o, d, m):
  line = LineModel([o, d])
  print "Creating line:  O=(", o[0], ", ", o[1], ")  D=(", d[0], ", ", d[1], ")"
  lm.wallIs(line, m)

def printWalls():
  print "Walls currently in the LocalMap:"
  for w in lm.walls:
    print "  O=(", w.origin()[0], ", ", w.origin()[1], ")  D=(", w.trajectory()[0], ", ", w.trajectory()[1], ")",
    print "  polar = (", w.polar[0], ", ", w.polar[1], ")"


if __name__ == "__main__":
#  pdb.set_trace()
  createLine( [-1,0], [-1,1], [-1,-1,-3,8] )
  createLine( [6,1], [6.01,0], [6,6.01,-3,8] )
  createLine( [-4,-3], [1,-3], [-1,6,-3,-3] )
  createLine( [5,8], [2,8.01], [-1,6,8,8.01] )
  createLine( [0,0], [1,1], [0,1,0,1] )
  createLine( [6,8], [6,12], [6,6,7,12] )
  printWalls()
