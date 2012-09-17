import numpy
import Image


img = Image.fromarray(numpy.random.rand(100,100) * 255)
img.convert('RGB').save("test.png")
