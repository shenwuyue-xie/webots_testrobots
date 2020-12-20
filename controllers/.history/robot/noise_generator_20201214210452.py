import numpy as np
import matplotlib.pyplot as plt

class OUActionNoise(object):
    def __init__(self, mu, sigma=0.15, theta=0.2, dt=1e-2, x0=None):
        self.mu = mu
        self.sigma = sigma
        self.theta = theta
        self.dt = dt
        self.x0 = x0
        self.reset()

    def __call__(self):
        x = self.x_prev + self.theta * (self.mu - self.x_prev) * self.dt + \
            self.sigma * np.sqrt(self.dt)*np.random.normal(size=self.mu.shape)
        self.x_prev = x
        return x

    def reset(self):
        self.x_prev = self.x0 if self.x0 is not None else np.zeros_like(
            self.mu)

def main():
	ou_noise=OUActionNoise(mu=np.zeros(2))
	plt.figure('data')
	y=[]
	t=np.linspace(0,100,1000)
	for _ in t:
		y.append(ou_noise())
	plt.plot(t,y)
	plt.show()
 
if __name__=="__main__":
	main()