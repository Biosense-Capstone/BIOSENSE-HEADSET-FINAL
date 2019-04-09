import numpy as np
import matplotlib.pyplot as plt
n = 100
X = np.linspace(0,2*np.pi,n)
Y = 40*np.cos(X) + 127.5
Y = Y + 40*np.cos(3*X) + 20 * np.cos(5*X)

fo = open("cos.c","w")
fo.write('#include "cos.h"\n\n')
fo.write("const uint8_t cosVal[{}] = {{\n".format(n))
for i, val in enumerate(Y, start = 1):
    fo.write("\t{},\n".format(int(val)))

fo.write("};\n")
fo.close()
plt.plot(X,Y)
plt.show()


