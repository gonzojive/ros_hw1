import math

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

def vector_length_squared(v):
    sum = 0
    for x in v:
        sum = sum + x*x
    return sum

def normalize(v):
    l = vector_length(v)
    return map(lambda x : x / l, v)

def vector_dot(v,w):
    return sum(map( lambda a,b: a * b, v, w))

def vector_angle(v, w):
    dot_result =  vector_dot(v,w)
    
    try:
	result = math.acos(dot_result)
 	return result
    except:
        # math domain error
	return 0
