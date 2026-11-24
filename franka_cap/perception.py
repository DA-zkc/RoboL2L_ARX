"""Geometry-based perception; no implicit remote model calls."""
import cv2
import numpy as np
from .geometry import matrix, vector


def localize_pixel(camera, pixel, radius=2):
    if not camera["depth_available"]:
        raise ValueError("This camera has no depth in the current SDK; RGB alone cannot produce a 3D point")
    uv = vector(pixel, 2, "pixel")
    if not np.equal(uv, np.round(uv)).all(): raise ValueError("Pixel indices must be integers")
    x, y = map(int, uv)
    if not 0 <= x < camera["width"] or not 0 <= y < camera["height"]:
        raise ValueError("Pixel outside image")
    if type(radius) is not int or not 0 <= radius <= 5: raise ValueError("radius must be an integer in 0..5")
    depth = np.load(camera["depth_path"], allow_pickle=False)
    roi = depth[max(0,y-radius):y+radius+1,max(0,x-radius):x+radius+1]
    valid = roi[np.isfinite(roi) & (roi > 0)]
    if not len(valid): raise ValueError("No valid depth at requested pixel")
    k = np.array([[camera['fx'],0,camera['ppx']],[0,camera['fy'],camera['ppy']],[0,0,1.]])
    coefficients = np.asarray(camera['coeffs'], dtype=float)
    if not np.isfinite(coefficients).all(): raise ValueError('Invalid distortion coefficients')
    if np.all(coefficients == 0):
        # D435 reports inverse_brown_conrady even for the rectified zero-distortion color stream.
        uvn = np.array([(x-camera['ppx'])/camera['fx'], (y-camera['ppy'])/camera['fy']])
    elif camera.get("model") == "distortion.brown_conrady":
        uvn = cv2.undistortPoints(np.array([[[x,y]]],dtype=float),k,coefficients)[0,0]
    else:
        raise ValueError("Nonzero unsupported distortion for deprojection")
    z = float(np.median(valid)); point = np.array([uvn[0]*z,uvn[1]*z,z,1.])
    base = matrix(camera["T_base_camera"]) @ point
    return {"pixel": [x,y], "point_camera_m": point[:3].tolist(), "point_base_m": base[:3].tolist(),
            "depth_m": z, "depth_valid_samples": int(len(valid)), "frame": "base",
            "meaning": "visible surface estimate, not an object centre or grasp pose"}


def color_regions(camera, color):
    ranges = {"red": [(0,10),(170,179)], "green": [(35,85)], "blue": [(90,135)], "orange": [(10,25)]}
    if color not in ranges: raise ValueError("Supported color diagnostics: red, green, blue, orange")
    image = cv2.imread(camera["rgb_path"])
    if image is None: raise ValueError("Observation image missing")
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV); mask = np.zeros(image.shape[:2],np.uint8)
    for lo,hi in ranges[color]: mask |= cv2.inRange(hsv,np.array([lo,90,50]),np.array([hi,255,255]))
    n, labels, stats, centers = cv2.connectedComponentsWithStats(mask)
    result=[]
    for i in range(1,n):
        x,y,w,h,area = stats[i]
        if area < 40: continue
        # Pick a pixel belonging to this component, not a centroid falling into a hole.
        ys,xs=np.where(labels==i); j=np.argmin((xs-centers[i,0])**2+(ys-centers[i,1])**2)
        item={"color":color,"area_px":int(area),"bbox_xywh":list(map(int,(x,y,w,h))),"pixel":[int(xs[j]),int(ys[j])],
              "semantic_identity":"unverified_color_region"}
        result.append(item)
    return sorted(result,key=lambda x:-x['area_px'])
