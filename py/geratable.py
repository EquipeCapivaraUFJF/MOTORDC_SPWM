import matplotlib.pyplot as plt
import numpy as np 

class Pwm():
    '''Generate senoidal reference for sPWM '''
    def __init__(self):
        self.top = 100
        self.len = 20
        self._vecSin = np.zeros(self.len,dtype=int)
    
    def gera_pwm(self):
        for i in range(0,self.len):
            self._vecSin[i] = int(np.sin(2*np.pi*i/self.len)*(1/2)*self.top)+(1/2)*self.top
            i=i+1
        return self._vecSin
        
if __name__=='__main__':
    vec=np.array(Pwm().gera_pwm())
    plt.plot(vec)
    plt.show()
    print(vec)