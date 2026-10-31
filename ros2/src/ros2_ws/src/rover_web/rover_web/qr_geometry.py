"""Bounded finder-pattern localization; geometry is NOT decoded QR text."""
from itertools import combinations
import cv2
import numpy as np


def locate_qr(frame, status=None):
    if status is not None:
        status['ambiguous'] = False
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
    # Bound work independently of camera resolution.
    scale = min(1., 1280. / max(gray.shape))
    if scale < 1:
        gray = cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    candidates = []
    mask = cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY_INV,31,5)
    contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    if hierarchy is None:
        return None
    for index, contour in enumerate(contours):
        child = hierarchy[0,index,2]
        if child < 0:
            continue
        core = hierarchy[0,child,2]
        if core < 0:
            continue
        area = abs(cv2.contourArea(contour))
        if not 12 <= area <= gray.size*.04:
            continue
        hole_area = abs(cv2.contourArea(contours[child]))
        core_area = abs(cv2.contourArea(contours[core]))
        if not (.30 < hole_area/area < .85 and .08 < core_area/area < .40):
            continue
        polygon = cv2.approxPolyDP(contour,.045*cv2.arcLength(contour,True),True)
        if len(polygon)!=4 or not cv2.isContourConvex(polygon):
            continue
        sides = np.linalg.norm(polygon[:,0]-np.roll(polygon[:,0],1,axis=0),axis=1)
        if sides.min() < 3 or sides.max()/sides.min() > 1.8:
            continue
        center = polygon[:,0].mean(axis=0)
        if np.linalg.norm(center-contours[core][:,0].mean(axis=0)) > sides.mean()*.3:
            continue
        candidates.append((center,float(sides.mean()),area))
    # Large real finder patterns take precedence over tiny background glyphs.
    candidates.sort(key=lambda item:item[2],reverse=True)
    candidates = candidates[:12]
    choices = []
    for group in combinations(candidates,3):
        centers = np.array([v[0] for v in group])
        sizes = np.array([v[1] for v in group])
        if sizes.max()/sizes.min() > 1.65:
            continue
        for i in range(3):
            a,b,c=centers[i],centers[(i+1)%3],centers[(i+2)%3]
            u,v=b-a,c-a
            lu,lv=np.linalg.norm(u),np.linalg.norm(v)
            cosine=abs(float(u@v))/max(1.,float(lu*lv))
            if cosine>.22 or min(lu,lv)/max(lu,lv)<.65:
                continue
            if not 1.7 < min(lu,lv)/sizes.mean() < 25:
                continue
            if u[0]*v[1]-u[1]*v[0]<0:
                u,v=v,u
            alpha=3.5/max(14.,7*(lu+lv)/(2*sizes.mean()))
            points=np.array([a-alpha*(u+v),a+(1+alpha)*u-alpha*v,
                             a+(1+alpha)*(u+v),a-alpha*u+(1+alpha)*v],np.float32)/scale
            if (points.min()< -8 or points[:,0].max()>frame.shape[1]+8
                    or points[:,1].max()>frame.shape[0]+8):
                continue
            lo,hi=points.min(0)*scale,points.max(0)*scale
            margin=(hi-lo)*.2
            x,y=np.maximum(0,(lo+margin).astype(int))
            r,b=np.minimum([gray.shape[1],gray.shape[0]],(hi-margin).astype(int))
            interior=gray[y:b,x:r]
            if not interior.size or interior.std()<20 or np.mean(cv2.Canny(interior,35,95)>0)<.10:
                continue
            choices.append((float(lu*lv)*(1-cosine),points))
    if not choices:
        return None
    choices.sort(key=lambda item:item[0],reverse=True)
    best=choices[0][1]
    for _,points in choices[1:]:
        if np.linalg.norm(best.mean(0)-points.mean(0))>max(np.ptp(best[:,0]),np.ptp(points[:,0]))*.6:
            if status is not None:
                status['ambiguous'] = True
            return None  # More than one plausible QR: don't select arbitrarily.
    return best
