import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pdb
import svm
from log import LogData
from constants import (GROUND, FACADE)


if __name__ == '__main__':
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    log1 = LogData('oakland_part3_am_rf.node_features')
    log2 = LogData('oakland_part3_an_rf.node_features')

    
    # run support vector code
    svm = svm.SVM(log1, (GROUND, FACADE))
    test_classes = log2.filter_data((GROUND, FACADE))
    svm.predict_and_plot(test_classes, ax)



    plt.show()
