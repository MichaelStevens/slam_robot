import numpy as np
from scipy.io import loadmat
from mlabwrap import mlab
import caffe


def init_detector():
    mlab.addpath('../external/caffe');
    mlab.addpath('../external/torque');
    mlab.addpath('../external/liblinear');

    mlab.matcaffe_init(False, '../data/deploy.prototxt', '../data/bvlc_reference_caffenet.caffemodel')
    print "done!"

def detect_mugs(I):
    return mlab.detect_mugs(I)
