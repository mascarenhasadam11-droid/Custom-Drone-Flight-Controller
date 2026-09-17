import receiver

Integral_P = 0
Integral_R = 0
error_old_P = 0
error_old_R = 0
error_new = 0

def PID(desired, actual, Integral, error_new, K1, K2, K3):   #Run with X and Y separately 
    error_old = error_new
    error_new = desired - actual
    
    #K1 K2 K3 gains to be edited
    
    Proportional = K1 * error_new
    Integral += K2 * error_new * dt
    Differential = (K3 * (error_new - error_old)) / dt
    
    PID = Proportional + Integral + Differential
    
    return(PID, Integral, error_new)
    

def mixer(throttle, pitch, roll): #Correction required for pitched down and rolled to right
    
    M1 = throttle + pitch - roll
    M2 = throttle + pitch + roll
    M3 = throttle - pitch - roll
    M4 = throttle - pitch + roll
    
    M1 = max(48, min(2047, M1))
    M2 = max(48, min(2047, M2))
    M3 = max(48, min(2047, M3))
    M4 = max(48, min(2047, M4))
    
    return M1,M2,M3,M4


