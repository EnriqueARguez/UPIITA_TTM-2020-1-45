import time
import matplotlib.pyplot
import cv2
import numpy as np
import os
from math import pow, atan2, sqrt, pi, acos, cos, sin, radians
from sklearn.neighbors import NearestNeighbors

def best_fit_transform(A, B):
    '''
    Calculates the least-squares best-fit transform that maps corresponding points A to B in m spatial dimensions
    Input:
      A: Nxm numpy array of corresponding points
      B: Nxm numpy array of corresponding points
    Returns:
      T: (m+1)x(m+1) homogeneous transformation matrix that maps A on to B
      R: mxm rotation matrix
      t: mx1 translation vector
    '''

    assert A.shape == B.shape

    # get number of dimensions
    m = A.shape[1]

    # translate points to their centroids
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    AA = A - centroid_A
    BB = B - centroid_B

    # rotation matrix
    H = np.dot(AA.T, BB)
    U, S, Vt = np.linalg.svd(H)
    R = np.dot(Vt.T, U.T)

    # special reflection case
    if np.linalg.det(R) < 0:
       Vt[m-1,:] *= -1
       R = np.dot(Vt.T, U.T)

    # translation
    t = centroid_B.T - np.dot(R,centroid_A.T)

    # homogeneous transformation
    T = np.identity(m+1)
    T[:m, :m] = R
    T[:m, m] = t

    return T, R, t


def nearest_neighbor(src, dst):
    '''
    Find the nearest (Euclidean) neighbor in dst for each point in src
    Input:
        src: Nxm array of points
        dst: Nxm array of points
    Output:
        distances: Euclidean distances of the nearest neighbor
        indices: dst indices of the nearest neighbor
    '''

    assert src.shape == dst.shape

    neigh = NearestNeighbors(n_neighbors=1)
    neigh.fit(dst)
    distances, indices = neigh.kneighbors(src, return_distance=True)
    return distances.ravel(), indices.ravel()


def icp(A, B, init_pose=None, max_iterations=20, tolerance=0.001, verbose = False):
    '''
    The Iterative Closest Point method: finds best-fit transform that maps points A on to points B
    Input:
        A: Nxm numpy array of data point cloud
        B: Nxm numpy array of template point cloud
        init_pose: (m+1)x(m+1) homogeneous transformation
        max_iterations: exit algorithm after max_iterations
        tolerance: convergence criteria
    Output:
        T: final homogeneous transformation that maps A on to B
        distances: Euclidean distances (errors) of the nearest neighbor
        i: number of iterations to converge
    '''

    assert A.shape == B.shape

    # get number of dimensions
    m = A.shape[1]

    # make points homogeneous, copy them to maintain the originals
    src = np.ones((m+1,A.shape[0]))
    dst = np.ones((m+1,B.shape[0]))
    src[:m,:] = np.copy(A.T)
    dst[:m,:] = np.copy(B.T)

    # apply the initial pose estimation
    if init_pose is not None:
        src = np.dot(init_pose, src)

    prev_error = 0
    Token = False
    for i in range(max_iterations):

        if verbose:
            print('----- Iteration Number ',i,' -----')
        # find the nearest neighbors between the current source and destination points
        distances, indices = nearest_neighbor(src[:m,:].T, dst[:m,:].T)

        # compute the transformation between the current source and nearest destination points
        T,_,_ = best_fit_transform(src[:m,:].T, dst[:m,indices].T)

        mean_error = np.mean(distances)
        # if np.abs(prev_error - mean_error) < tolerance and mean_error > 0.014 and Token != True:
        #   Tr=np.array([[-1,0,0],[0,-1,0],[0,0,1]])
        #   T=T*Tr
        #   Token = True

        # update the current source
        src = np.dot(T, src)

        # check error
        if verbose:
            print('Error distancias: ',np.mean(distances))
        # if np.abs(prev_error - mean_error) < tolerance:
        #     break
        prev_error = mean_error

    # calculate final transformation
    T,_,_ = best_fit_transform(A, src[:m,:].T)

    return T, distances, i


def test_icp(data,template):

    pc_ref = np.copy(template.T)
    pc = np.copy(data.T)

    # Run ICP
    start = time.time()
    T, distances, iterations = icp(pc, pc_ref, max_iterations=100, tolerance=0.00001,verbose=False)
    print('T: {}'.format(T))

    return T

def plotresult(data,template,T):
    result = cv2.transform(np.array([data.T], copy=True).astype(np.float32), T).T
    matplotlib.pyplot.plot(template[0], template[1], label="template")
    matplotlib.pyplot.plot(data[0], data[1], label="data")
    matplotlib.pyplot.plot(result[0], result[1], label="result")
    matplotlib.pyplot.legend(loc="upper left")
    matplotlib.pyplot.axis('square')
    # matplotlib.pyplot.draw()
    # matplotlib.pyplot.pause(0.00000000001)
    matplotlib.pyplot.show()

def base_map(d, verbose = False):
    dir=os.getcwd()
    if not os.path.exists(dir+'/Mapas'):
        os.mkdir(dir+'/Mapas')

    if os.path.exists(dir+'/Mapas/Mapa_Base.txt'):
        os.remove(dir+'/Mapas/Mapa_Base.txt')

    x=[]
    y=[]

    for i in range(len(d)):
        x.append(str(d[i]*cos(radians(i))))
        y.append(str(d[i]*sin(radians(i))))

    X=" ".join(np.copy(x))
    Y=" ".join(np.copy(y))

    File=open(dir+'/Mapas/Mapa_Base.txt','w')
    File.write(X)
    File.write('\n')
    File.write(Y)
    File.close()
    
    if verbose == True:
        print('El mapa base ha sido guardado en el directorio: ', dir,'/Mapas')

def map(d,save_file = False):
    dir=os.getcwd()

    x=[]
    xr=[]
    y=[]
    yr=[]
    name=None

    for i in range(len(d)):
        x.append(str(d[i]*cos(radians(i))))
        xr.append(d[i]*cos(radians(i)))
        y.append(str(d[i]*sin(radians(i))))
        yr.append(d[i]*sin(radians(i)))

    X=" ".join(np.copy(x))
    Y=" ".join(np.copy(y))

    if save_file == True:
        name=raw_input('Ingrese el nombre del mapa a salvar: ')
        File=open(dir+'/Mapas/'+name+'.txt','w')
        File.write(X)
        File.write('\n')
        File.write(Y)
        File.close()

        return xr, yr, name
    else:
        return xr, yr, name

def plot_map(name):
    dir=os.getcwd()
    fdir=dir+'/Mapas/'+name+'.txt'

    File=open(fdir,'r')
    data=File.readlines()
    File.close()

    aux=[c.strip('\n') for c in data]
    auxx=aux[0].split(' ')
    auxy=aux[1].split(' ')

    x=[float(value) for value in auxx]
    y=[float(value) for value in auxy]

    matplotlib.pyplot.plot(x, y, label=name)

def del_map(name=None,del_all=False):
    dir=os.getcwd()
    mdir=dir+'/Mapas'

    texts=sorted(os.listdir(mdir))

    if del_all == True:
        for text in texts:
            os.remove(mdir+text)
    elif name != None:
        os.remove(mdir+name+'.txt')
    else:
        n=raw_input('Ingrese el nombre del mapa a eliminar: ')
        if not os.path.exists(mdir+n):
            print('El archivo no se encuentra o ya ha sido eliminado previamente..')
        else:
            os.remove(mdir+n+'.txt')