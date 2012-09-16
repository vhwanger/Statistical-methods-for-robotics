import pylab
import numpy
import pdb
import matplotlib.pyplot as plt

img = numpy.random.rand(100,100)
plt.imshow(img)
plt.axis('off')
plt.savefig('test.png')
