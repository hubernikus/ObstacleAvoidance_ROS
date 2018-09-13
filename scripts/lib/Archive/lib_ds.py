import numpy as np

def linearAttractor_const(x, x0 = 'default', velConst=0.3, distSlow=0.1):
    # change initial value for n dimensions
    # TODO -- constant velocity // maximum velocity
    
    dx = (x0-x)/distSlow
    dx_mag = np.sqrt(np.sum(dx**2))
    
    dx = min(1, 1/dx_mag)*velConst*dx

    return dx
 
