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
    return math.sqrt(sum)

def vector_length_squared(v):
    sum = 0
    for x in v:
        sum = sum + x*x
    return sum

def normalize(v):
    l = vector_length(v)
    return map(lambda x : x / l, v)

def vector_normalize(v):
    l = vector_length(v)
    return map(lambda x : x / l, v)

def vector_dot(v,w):
    return sum(map( lambda a,b: a * b, v, w))

def vector_angle_normalized(v, w):
    dot_result =  vector_dot(v,w)
    
    try:
	result = math.acos(dot_result)
 	return result
    except:
        # math domain error
	return 0

#oddly, this assumes normalized angles
def vector_angle(v, w):
    return vector_angle_normalized(v, w)

def vector_angle_general(v, w):
    return vector_angle_normalized(normalize(v),normalize(w))

def vector_angle_signed(v, w):
    #return math.atan2(w[1] - v[1], w[0] - v[0])
    
    ang = atan2(v[1],v[0]) -     atan2(w[1],w[0])
    # we are in a right-handed system so return the opposite sign
    return ang * -1.0

    #z = [
    #signed_angle = atan2(  N * ( V1 x V2 ), V1 * V2  );
    # where * is dot product and x is cross product
    # N is the normal to the polygon
    # ALL vectors: N, V1, V2 must be normalized

    #sign = (vector_dot(
    #    c = cross(y,z);
    #     angleyz = sign(dot(x,c))*atan2(norm(c),dot(y,z));
    #3angleyz = sign(dot(x,c))*atan2(norm(c),dot(y,z));
