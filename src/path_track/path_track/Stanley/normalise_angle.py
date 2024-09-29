import math as m

def normalise_angle(angle):
    
    angle = m.atan2(m.sin(angle), m.cos(angle))

    return angle


if __name__ == '__main__':
    print(normalise_angle(3.2))
