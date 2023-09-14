def distance(A: list[float], B: list[float]=[.0, .0]):
    """
    Euclidean distance in 2D space between points A and B specified as lists of coordinates. 
    If only one point passed, the length of the vector is returned.
    """
    try:
        return ((A[0] - B[0]) ** 2 + (A[1] - B[1]) ** 2) ** (1 / 2)
    except (TypeError):
        raise AssertionError("Input variables should be lists of numbers.")

# Angle between two points
def angle(A, B):
    """
    Angle (in radians) between the points B and A in 2D space, specified as lists of coordinates. 
    """
    from numpy import arctan2
    try:
        return arctan2(B[0] - A[0], B[1] - A[1])
    except (TypeError):
        raise AssertionError("Input variables should be lists of numbers.")

def float_to_uint8(color):
    """
    Transforms the RGB color in floating type to uint8 type.
    """
    return (int(color[0]*255), int(color[1]*255), int(color[2]*255))