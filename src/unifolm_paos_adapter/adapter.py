"""Convert common PAOS observations into the official UniFoLM server input."""
import numpy as np

CAMERA_ROLES={'head':'cam_left_high','left_wrist':'cam_left_wrist','right_wrist':'cam_right_wrist'}


class InputUnavailable(ValueError):pass


def build_observation(observation,*,kinematics,instruction,gripper_mapping):
    try:
        if not isinstance(instruction,str)or not instruction.strip()or len(instruction)>8192:
            raise InputUnavailable('explicit nonempty instruction required')
        if gripper_mapping['kind']!='dex1_native_rad':
            raise InputUnavailable('explicit supported Dex1 input profile required')
        robot=observation['robot'];state=robot['state']
        if state is None or not state['usable']or state['run_id']!=robot['run_id']:
            raise InputUnavailable('body state unavailable or from another run')
        q=np.asarray(state['q'])
        ee=kinematics.ee9(q)
        result={'instruction':instruction,'unnorm_key':'UnifoLM_G1_Dex1',
                'observation.state.lower_body':q[:15].astype(np.float32)}
        for side in ['left','right']:
            hand=robot['grippers'][side]
            if not hand['available']or hand['state']is None:raise InputUnavailable(f'{side} gripper unavailable')
            raw=hand['state']['q'];offset=gripper_mapping['offsets'][side]
            if isinstance(raw,bool)or isinstance(offset,bool)or not np.isfinite([raw,offset]).all():
                raise InputUnavailable('finite native gripper position/offset required')
            result[f'observation.state.{side}_ee_6d']=ee[side]
            result[f'observation.state.{side}_gripper']=np.array([float(raw)-float(offset)],np.float32)
        for name,role in CAMERA_ROLES.items():
            frame=observation['frames'][name]
            if frame is None or robot['cameras'][name]!='Green':raise InputUnavailable(f'{name} camera unavailable')
            if frame['metadata']['run_id']!=robot['run_id']or frame['metadata']['camera']!=name:
                raise InputUnavailable('camera run/role mismatch')
            h,w=frame['height'],frame['width']
            if type(h)is not int or type(w)is not int or not 0<h<=480 or not 0<w<=640 or frame['encoding']not in ('rgb8','jpeg'):
                raise InputUnavailable('unsupported RGB dimensions/encoding')
            rgb=frame_rgb(frame)
            # Official server consumes BGR and performs its own BGR->RGB/resize.
            result[f'observation.images.{role}']=rgb[:,:,::-1].copy()
        return result
    except InputUnavailable:raise
    except (KeyError,TypeError,ValueError)as exc:
        raise InputUnavailable(f'incomplete or invalid model input: {exc}')from exc


def frame_rgb(frame):
    h,w=frame['height'],frame['width']
    if frame['encoding']=='rgb8':
        raw=frame['rgb']
        if not isinstance(raw,bytes)or len(raw)!=h*w*3:raise InputUnavailable('RGB byte length mismatch')
        return np.frombuffer(raw,np.uint8).reshape(h,w,3)
    if frame['encoding']=='jpeg':
        from io import BytesIO
        from PIL import Image
        with Image.open(BytesIO(frame['data']))as image:
            if image.format!='JPEG'or image.size!=(w,h):raise InputUnavailable('JPEG format/dimensions mismatch')
            return np.asarray(image.convert('RGB'))
    raise InputUnavailable('unsupported image encoding')
