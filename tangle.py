import math
def trans2(angle, posx, posy):
    x = posx*math.cos(angle)-posy*math.sin(angle)
    y = posx*math.sin(angle)+posy*math.cos(angle)
    return x,y

