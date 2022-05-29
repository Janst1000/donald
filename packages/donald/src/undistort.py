import rospy
# TODO: remove this from here
from os.path import basename, isfile, splitext
from sys import exit

import cv2

import numpy as np

def remove_unicode(x):

    if isinstance(x, str):
        return x.encode('utf8')

    if isinstance(x, dict):
        T = type(x)
        return T([(remove_unicode(k), remove_unicode(v)) for k, v in list(x.items())])

    if isinstance(x, list):
        T = type(x)
        return T([remove_unicode(_) for _ in x])

    return x

def yaml_load(s):
    from ruamel import yaml

    if s.startswith('...'):
        return None
    try:
        l = yaml.load(s, Loader=yaml.RoundTripLoader)
    except:
        l = yaml.load(s, Loader=yaml.UnsafeLoader)

    return l

def load_camera_intrinsics(veh):
    path = '/calibrations/camera_'
    # change this to your robots calibration file
    filename = './donald_intrinsic.yaml'
    if not isfile(filename):
        print(('Intrinsic calibration for {} does not exist.'.format(veh)))
        exit(3)
    with open(filename) as f:
        contents = f.read()
        data = yaml_load(contents)
    intrinsics = {}
    intrinsics['K'] = np.array(data['camera_matrix']['data']).reshape(3, 3)
    intrinsics['D'] = np.array(data['distortion_coefficients']['data']).reshape(1, 5)
    intrinsics['R'] = np.array(data['rectification_matrix']['data']).reshape(3, 3)
    intrinsics['P'] = np.array(data['projection_matrix']['data']).reshape((3, 4))
    intrinsics['distortion_model'] = data['distortion_model']
    print('Loaded camera intrinsics for {}'.format(veh))
    return intrinsics

def rectify(image, intrinsics):
    '''Undistort image'''
    height, width, _ = image.shape
    rectified_image = np.zeros(np.shape(image))
    mapx = np.ndarray(shape=(height, width, 1), dtype='float32')
    mapy = np.ndarray(shape=(height, width, 1), dtype='float32')
    mapx, mapy = cv2.initUndistortRectifyMap(intrinsics['K'],
                                             intrinsics['D'],
                                             intrinsics['R'],
                                             intrinsics['P'],
                                             (width, height), cv2.CV_32FC1, mapx, mapy)
    return cv2.remap(image, mapx, mapy, cv2.INTER_CUBIC, rectified_image)