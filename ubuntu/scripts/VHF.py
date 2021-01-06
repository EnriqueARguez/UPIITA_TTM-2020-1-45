
from numpy import rad2deg, deg2rad, zeros, pi

def generate_histogram(data,yaw):
    min_dist = 25.0 #[cm]
    lidar_err = 0.004
    s = 30 # Grados a ensanchar el Histograma

    theta=convert2theta(yaw)

    H  = zeros(360,dtype='f')
    Ho = zeros(360,dtype='f')
    hp = zeros(360,dtype='f')

    for i in range(360):
        dist = data[obtainIndexAngle(i,theta)]*100
        if dist <= min_dist and dist >= lidar_err:
            H[i],Ho[i] = [1,1] # Guarda el dato como obstaculo
        else:
            H[i],Ho[i] = [0,0] # Guarda el dato como espacio libre

    crest = [] #Crestas
    trough = [] #Valles

    for i in range(360):
        hp[i] = H[i] - H[(i - 1) % 360] # Derivada del histograma
        if hp[i] > 0.5:
            crest.append(i) #Guarda el angulo donde se encuentra la cresta
        elif hp[i] < -0.5:
            trough.append(i) #Guarda el angulo donde se encuentra el valle

    l = len(crest)
    if l > 0:
        for i in range(l):
            ind_back = crest[i]
            for k in range(s): # Ensancha la cresta "s" grados hacia la izquierda
                H[(ind_back - k) % 360] = 1
    l = len(trough)
    if l > 0:
        for i in range(l):
            ind_forw = trough[i]
            for k in range(s): # Ensancha los valles "s" grados hacia la derecha
                H[(ind_forw + k) % 360] = 1
    return H, Ho

def obtainIndexAngle(angle,theta):
    return (angle-int(theta))%360 # Ajusta el angulo al marco de referencia absoluto

def convert2theta(yaw):
    if yaw < 0:
        yaw += 2*pi
    return rad2deg(yaw)