import numpy as np

class Buffer:

  def __init__(self,max_size,window):
    self.max=max_size
    self.window=window
    self.data= np.array([])
    self.latest=0

  def append(self,x):

    self.data=np.append(self.data,x)
    if len(self.data)==self.max:
      self.cur=0
      self.data= np.array([])

  def get(self):
    return self.data

  def running_mean(self):
    try:
      mean = np.convolve(self.data, np.ones((self.window,))/self.window, mode='valid')
      self.latest=mean[-1]
      return mean
    except ValueError:
      return np.array([self.latest])


    

