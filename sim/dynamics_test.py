from dynamics import *

#create a bicycle object
bicycle = Bicycle()
#example state and input
x_0 = np.zeros((7, 1))
u_0 = np.zeros((4, 1))
t = 0
print("Next step: ", bicycle.f(x_0, u_0, t))
