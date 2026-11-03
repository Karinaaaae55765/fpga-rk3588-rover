"""Measured ground-plane projection only. No guessed focal lengths or object sizes."""
import math

def inside(point,polygon):
    if len(polygon)<3:return False
    x,y=point;result=False
    for (a,b),(c,d) in zip(polygon,polygon[1:]+polygon[:1]):
        if (b>y)!=(d>y) and x<(c-a)*(y-b)/(d-b)+a:result=not result
    return result

def ready(cal,require_start_pose=True):
    keys=('vehicle_verified','ground_projection_verified','recognition_envelope_verified',
          'tof_extrinsics_verified')
    if require_start_pose:keys+=('start_pose_verified',)
    if not all(cal.get(k) is True for k in keys):raise ValueError('physical calibration not verified')
    if not cal.get('evidence') or str(cal['evidence']).startswith('UNMEASURED'):
        raise ValueError('calibration evidence missing')
    h=cal.get('pixel_to_robot_ground_h')
    if not isinstance(h,list) or len(h)!=3 or any(not isinstance(row,list) or len(row)!=3 for row in h):
        raise ValueError('measured ground homography missing')
    if not all(math.isfinite(v) for row in h for v in row):raise ValueError('nonfinite homography')
    det=(h[0][0]*(h[1][1]*h[2][2]-h[1][2]*h[2][1])-
         h[0][1]*(h[1][0]*h[2][2]-h[1][2]*h[2][0])+
         h[0][2]*(h[1][0]*h[2][1]-h[1][1]*h[2][0]))
    if abs(det)<1e-12:raise ValueError('singular homography')
    error=cal.get('max_holdout_error_m')
    if error is None or not math.isfinite(error) or not 0<=error<=cal['allowed_holdout_error_m']:
        raise ValueError('independent ground validation failed')
    if len(cal['valid_ground_polygon_px'])<3:raise ValueError('measured ground region missing')
    if not 0<cal['recognition_min_m']<cal['recognition_max_m'] or not 0<cal['recognition_half_fov_rad']<math.pi/2:
        raise ValueError('invalid measured recognition envelope')

class GroundProjector:
    def __init__(self,cal):self.cal=cal

    def project(self,item,objects=()):
        c=self.cal
        if not c.get('ground_projection_verified'):return None
        left,top,right,bottom=item['box']; u=(left+right)/2;v=bottom
        if min(left,top)<3 or right>c['image_width']-3 or bottom>c['image_height']-3:return None
        # Reject another detection crossing the supposed floor-contact point.
        for other in objects:
            if other is item or other.get('index')==item.get('index'):continue
            a,b,d,e=other['box']
            if a<=u<=d and b<=v<=e:return None
        if not inside((u,v),c['valid_ground_polygon_px']):return None
        if c.get('camera_matrix') is not None:
            import cv2,numpy as np
            k=np.asarray(c['camera_matrix'],dtype=float)
            u,v=cv2.undistortPoints(np.asarray([[[u,v]]],dtype=float),k,
                     np.asarray(c['distortion_coefficients'],dtype=float),P=k)[0,0]
        h=c['pixel_to_robot_ground_h']
        if h is None:return None
        z=h[2][0]*u+h[2][1]*v+h[2][2]
        if abs(z)<1e-9:return None
        x=(h[0][0]*u+h[0][1]*v+h[0][2])/z
        y=(h[1][0]*u+h[1][1]*v+h[1][2])/z
        distance=math.hypot(x,y)
        if not all(math.isfinite(t) for t in (x,y)) or x<=0 or not c['recognition_min_m']<=distance<=c['recognition_max_m']:return None
        return x,y

    def range(self,item,objects,tof_m,tof_offset):
        p=self.project(item,objects)
        if p is None:return None
        x,y=p
        # A point projection is not proof that the ToF beam hit that object.
        # Agreement is a consistency check, not identity verification.
        if abs(math.atan2(y,x))<=self.cal['tof_association_half_angle_rad']:
            if tof_m is None or abs(x-(tof_m+tof_offset))>self.cal['tof_agreement_m']:return None
        return dict(x=x,y=y,distance=math.hypot(x,y),bearing=math.atan2(y,x),
                    method='calibrated_ground_contact',identity_verified=False)

def fit_ground(pixels,ground,holdout_pixels,holdout_ground):
    """Return measured H and independent error; never set verification flags."""
    import cv2,numpy as np
    if len(pixels)<6 or len(holdout_pixels)<4:raise ValueError('need >=6 fit and >=4 independent points')
    h,_=cv2.findHomography(np.asarray(pixels,dtype=float),np.asarray(ground,dtype=float),0)
    if h is None:raise ValueError('degenerate calibration points')
    predicted=cv2.perspectiveTransform(np.asarray([holdout_pixels],dtype=float),h)[0]
    error=float(np.linalg.norm(predicted-np.asarray(holdout_ground),axis=1).max())
    return dict(pixel_to_robot_ground_h=h.tolist(),max_holdout_error_m=error)
