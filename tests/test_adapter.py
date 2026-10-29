import math
import numpy as np
import pytest
from unifolm_paos_adapter.kinematics import G1Kinematics
from unifolm_paos_adapter.adapter import build_observation, InputUnavailable
from unifolm_paos_adapter.chunk import ModelActionChunk, base_goal_at, FIELD_DIMS
from unifolm_paos_adapter._vendor import msgpack_numpy


def snapshot():
    frames={name:dict(width=2,height=1,encoding='rgb8',rgb=bytes([1,2,3,10,20,30]),
                     metadata=dict(run_id=73,camera=name))for name in ['head','left_wrist','right_wrist']}
    robot=dict(run_id=73,state=dict(run_id=73,usable=True,q=[0.]*29),
        cameras={n:'Green'for n in frames},grippers={
        'left':dict(available=True,state=dict(q=2.7,opening=.5)),
        'right':dict(available=True,state=dict(q=5.4,opening=1.))})
    return dict(robot=robot,frames=frames)


def raw_chunk(offset=0):
    result={k:np.zeros((1,30,d),np.float32)for k,d in FIELD_DIMS.items()}
    result['action.base_command'][0,:,0]=np.arange(30)+offset
    return result


def test_official_urdf_zero_pose_and_full_waist_chain():
    k=G1Kinematics();before=k.ee9(np.zeros(29))
    # Independent XML-chain zero-pose calculation; no geometry overrides.
    np.testing.assert_allclose(before['left'][:3],[.30977428265902246,.14864062926613938,.0952267873553472],atol=1e-9)
    np.testing.assert_allclose(before['right'][:3],[.30977428265902246,-.14863062926613937,.0952267873553472],atol=1e-9)
    q=np.zeros(29);q[12]=.4;after=k.ee9(q)
    r=np.array([[math.cos(.4),-math.sin(.4),0],[math.sin(.4),math.cos(.4),0],[0,0,1]])
    for side in ['left','right']:
        for start in [0,3,6]:np.testing.assert_allclose(after[side][start:start+3],r@before[side][start:start+3],atol=1e-7)
    with pytest.raises(ValueError):k.ee9(np.zeros(28))
    q[0]=np.nan
    with pytest.raises(ValueError):k.ee9(q)
    with pytest.raises(ValueError):G1Kinematics(joint_names=['duplicate']*29)


def test_observation_bgr_ee_native_rad_and_no_double_normalization():
    value=build_observation(snapshot(),kinematics=G1Kinematics(),instruction='walk forward',
                            gripper_mapping={'kind':'dex1_native_rad','offsets':{'left':0.,'right':0.}})
    assert value['observation.images.cam_left_high'].tolist()==[[[3,2,1],[30,20,10]]]
    assert value['observation.state.left_ee_6d'].shape==(9,)
    assert value['observation.state.left_gripper'].tolist()==pytest.approx([2.7])
    assert value['observation.state.right_gripper'].tolist()==pytest.approx([5.4])
    assert value['observation.state.lower_body'].shape==(15,)
    assert value['instruction']=='walk forward' and value['unnorm_key']=='UnifoLM_G1_Dex1'
    restored=msgpack_numpy.unpackb(msgpack_numpy.packb(value))
    np.testing.assert_array_equal(restored['observation.images.cam_left_high'],value['observation.images.cam_left_high'])


def snapshot_with_rgb_frames(width,height):
    data=snapshot();pixels={}
    rows,columns=np.indices((height,width))
    for index,name in enumerate(['head','left_wrist','right_wrist']):
        rgb=np.empty((height,width,3),np.uint8)
        rgb[:,:,0]=(columns+index)%256
        rgb[:,:,1]=(rows+7*index)%256
        rgb[:,:,2]=(rows+columns+31*index)%256
        rgb[0,0]=[11+index,22+index,33+index]
        rgb[-1,-1]=[44+index,55+index,66+index]
        data['frames'][name].update(width=width,height=height,rgb=rgb.tobytes())
        pixels[name]=rgb
    return data,pixels


@pytest.mark.parametrize('width,height',[(320,240),(640,480)])
def test_raw_qvga_and_vga_preserve_all_camera_pixels_and_roles(width,height):
    data,pixels=snapshot_with_rgb_frames(width,height)
    value=build_observation(data,kinematics=G1Kinematics(),instruction='pick carrot',
        gripper_mapping={'kind':'dex1_native_rad','offsets':{'left':0.,'right':0.}})
    for index,(name,role)in enumerate([('head','cam_left_high'),
            ('left_wrist','cam_left_wrist'),('right_wrist','cam_right_wrist')]):
        image=value[f'observation.images.{role}']
        assert image.shape==(height,width,3) and image.dtype==np.uint8
        assert image.flags.c_contiguous
        assert image[0,0].tolist()==[33+index,22+index,11+index]
        assert image[-1,-1].tolist()==[66+index,55+index,44+index]
        np.testing.assert_array_equal(image[:,:,0],pixels[name][:,:,2])
        np.testing.assert_array_equal(image[:,:,1],pixels[name][:,:,1])
        np.testing.assert_array_equal(image[:,:,2],pixels[name][:,:,0])


@pytest.mark.parametrize('instruction',['pick carrot','🥕'*8192],ids=['short','max-utf8'])
def test_three_raw_vga_model_request_fits_websocket_limit(instruction):
    data,_=snapshot_with_rgb_frames(640,480)
    value=build_observation(data,kinematics=G1Kinematics(),instruction=instruction,
        gripper_mapping={'kind':'dex1_native_rad','offsets':{'left':0.,'right':0.}})
    packed=msgpack_numpy.packb({'type':'get_action','obs':value})
    assert sum(image.nbytes for key,image in value.items()if key.startswith('observation.images.'))==2_764_800
    assert len(packed)<4*1024*1024
    restored=msgpack_numpy.unpackb(packed)
    assert restored['type']=='get_action' and restored['obs']['instruction']==instruction
    for role in ['cam_left_high','cam_left_wrist','cam_right_wrist']:
        key=f'observation.images.{role}'
        np.testing.assert_array_equal(restored['obs'][key],value[key])


@pytest.mark.parametrize('name',['head','left_wrist','right_wrist'])
@pytest.mark.parametrize('width,height',[(641,480),(640,481)])
def test_camera_dimensions_above_vga_are_rejected(name,width,height):
    data=snapshot()
    data['frames'][name].update(width=width,height=height,rgb=bytes(width*height*3))
    with pytest.raises(InputUnavailable,match='unsupported RGB dimensions/encoding'):
        build_observation(data,kinematics=G1Kinematics(),instruction='pick carrot',
            gripper_mapping={'kind':'dex1_native_rad','offsets':{'left':0.,'right':0.}})


@pytest.mark.parametrize('kind',['missing_frame','red_state','red_gripper','wrong_run','bad_buffer'])
def test_incomplete_inputs_are_not_fabricated(kind):
    data=snapshot()
    if kind=='missing_frame':data['frames']['head']=None
    elif kind=='red_state':data['robot']['state']['usable']=False
    elif kind=='red_gripper':data['robot']['grippers']['left']['available']=False
    elif kind=='wrong_run':data['frames']['head']['metadata']['run_id']=74
    else:data['frames']['head']['rgb']=b''
    with pytest.raises(InputUnavailable):build_observation(data,kinematics=G1Kinematics(),instruction='walk',gripper_mapping={'kind':'dex1_native_rad','offsets':{'left':0.,'right':0.}})


def test_chunk_preserves_all_fields_and_time_boundaries():
    raw=raw_chunk();chunk=ModelActionChunk.from_response(raw,received_at=10.,period_s=1/30)
    assert set(chunk.actions)==set(FIELD_DIMS)
    assert all(chunk.actions[k].dtype==raw[k].dtype for k in raw)
    for i in [0,1,10,29]:assert base_goal_at(chunk,elapsed_s=i/30)==(float(i),0.,0.)
    assert base_goal_at(chunk,elapsed_s=1.)is None
    raw['action.base_command'][0,0,0]=999
    assert base_goal_at(chunk,elapsed_s=0)==(0.,0.,0.)
    for bad_time in [-1,float('nan')]:
        with pytest.raises(ValueError):base_goal_at(chunk,elapsed_s=bad_time)
    bad=raw_chunk();bad['action.left_gripper'][0,0,0]=np.nan
    with pytest.raises(ValueError):ModelActionChunk.from_response(bad,received_at=0,period_s=1/30)
    bad=raw_chunk();del bad['action.pivot']
    with pytest.raises(ValueError):ModelActionChunk.from_response(bad,received_at=0,period_s=1/30)


def test_explicit_jpeg_transport_is_decoded_before_official_bgr_input():
    from io import BytesIO
    from PIL import Image
    data=snapshot();buffer=BytesIO()
    Image.fromarray(np.full((1,2,3),[17,83,201],dtype=np.uint8)).save(buffer,format='JPEG',quality=95)
    for f in data['frames'].values():
        f.pop('rgb');f['data']=buffer.getvalue();f['encoding']='jpeg'
    result=build_observation(data,kinematics=G1Kinematics(),instruction='pick carrot',
        gripper_mapping={'kind':'dex1_native_rad','offsets':{'left':0.,'right':0.}})
    assert result['observation.images.cam_left_high'].dtype==np.uint8
    np.testing.assert_allclose(result['observation.images.cam_left_high'][0,0],[201,83,17],atol=3)
