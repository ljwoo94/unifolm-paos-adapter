"""Model-only RTC mechanism/memory comparison; never connects to a robot.

Fixed recorded observation, synthetic schedule s=15,d=10 at native 30Hz.
Guidance: original RTC equations 2-5 (arxiv:2506.07339v1).
STE is approximate; dequantized-head weights are NOT original BF16 weights.
This experiment does not establish closed-loop success or real-time viability.
"""
import argparse
import gc
import json
import math
import os
from pathlib import Path
import sys
import time

os.environ['WMA_TORCH_COMPILE']='0'
import numpy as np
import torch
from torch import nn
import torch.nn.functional as F

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'model_runtime'))
from runtime import ModelRuntime,BUNDLE
sys.path.insert(0,str(BUNDLE))
from native_int8 import W8A8Linear


def memory():
    return dict(allocated=torch.cuda.memory_allocated(),reserved=torch.cuda.memory_reserved(),
                peak_allocated=torch.cuda.max_memory_allocated(),peak_reserved=torch.cuda.max_memory_reserved())


class STE(torch.autograd.Function):
    @staticmethod
    def forward(ctx,x,layer):
        ctx.save_for_backward(layer.qweight_t,layer.scales,layer.pre_scale)
        ctx.shape=x.shape;ctx.i=layer.in_features;ctx.o=layer.out_features
        return ORIGINAL_LINEAR(layer,x)

    @staticmethod
    def backward(ctx,grad):
        q,s,pre=ctx.saved_tensors
        # Approximate activation quantizer with identity derivative; fixed weights.
        w=q[:ctx.i,:ctx.o].to(torch.bfloat16)
        g=(grad.reshape(-1,ctx.o).float()*s[:ctx.o]).to(torch.bfloat16)
        dx=(g@w.t()).float()*pre.float()
        return dx.reshape(ctx.shape).to(grad.dtype),None


ORIGINAL_LINEAR=W8A8Linear.forward
AMP_ENABLED=False
AMP_DTYPE=torch.bfloat16


def ste_forward(layer,x):
    if torch.is_grad_enabled()and x.requires_grad:return STE.apply(x,layer)
    return ORIGINAL_LINEAR(layer,x)


class DenseHeadLinear(nn.Module):
    def __init__(self,old):
        super().__init__()
        weight=(old.qweight_t[:old.in_features,:old.out_features].t().float()*old.scales[:old.out_features,None]).to(torch.bfloat16)
        self.weight=nn.Parameter(weight,requires_grad=False)
        self.register_buffer('pre_scale',old.pre_scale)
        self.register_buffer('bias',old.bias)
    def forward(self,x):
        return F.linear((x*self.pre_scale).to(torch.bfloat16),self.weight,self.bias).to(x.dtype)


def dequantize_head(head):
    names=[n for n,m in head.named_modules()if isinstance(m,W8A8Linear)]
    for name in names:
        old=head.get_submodule(name);new=DenseHeadLinear(old)
        parent,_,leaf=name.rpartition('.')
        setattr(head.get_submodule(parent)if parent else head,leaf,new)
        del old,new
    gc.collect();torch.cuda.empty_cache()
    return len(names)


def velocity(head,ctx,actions,t):
    vl,state,mask,keymask,body=ctx
    mp=mask.float().unsqueeze(1).expand(-1,head.action_horizon,-1).contiguous()
    ts=torch.full((actions.shape[0],),int(t*head.num_timestep_buckets),device=actions.device,dtype=torch.long)
    features=head.action_encoder(torch.cat([actions,mp],dim=2),ts)
    features=head._add_action_pos_embed(features,actions.device)
    features=head._add_embodiment_embed(features,body)
    sf=head.state_encoder(state)if head.state_encoder is not None and state is not None else None
    hidden=torch.cat([sf,features],dim=1)if sf is not None else features
    y=head.model(hidden_states=hidden,encoder_hidden_states=vl,encoder_attention_mask=keymask,timestep=ts)
    y=head._add_embodiment_embed_dec(y,body)
    return head.action_decoder(y)[:,-head.action_horizon:]


def sample(head,ctx,seed,prior=None,beta=0.,d=10,s=15):
    with torch.autocast('cuda',enabled=AMP_ENABLED,dtype=AMP_DTYPE):
        return _sample(head,ctx,seed,prior,beta,d,s)


def _sample(head,ctx,seed,prior=None,beta=0.,d=10,s=15):
    vl,_,mask,_,_=ctx
    torch.manual_seed(seed);torch.cuda.manual_seed_all(seed)
    actions=torch.randn((1,30,54),device=vl.device,dtype=vl.dtype)
    mp=mask.float()[:,None,:].expand(-1,30,-1).contiguous()
    actions=actions*mp
    w=torch.zeros((1,30,1),device=vl.device)
    for i in range(30):
        if i<d:w[:,i]=1.
        elif i<30-s:
            c=(30-s-i)/(30-s-d+1);w[:,i]=c*math.expm1(c)/math.expm1(1.)
    torch.cuda.synchronize();begun=time.perf_counter()
    for k in range(4):
        t=k/4
        if prior is None or beta==0:
            with torch.no_grad():v=velocity(head,ctx,actions,t)
        else:
            with torch.enable_grad():
                x=actions.detach().requires_grad_(True)
                v=velocity(head,ctx,x,t)
                estimate=x+(1-t)*v
                error=((prior-estimate)*w*mp).detach()
                vjp=torch.autograd.grad(estimate,x,grad_outputs=error)[0]
                weight=beta if t==0 else min(beta,(t*t+(1-t)**2)/(t*(1-t)))
                v=v.detach()+weight*vjp.detach()
        actions=((actions+v/4)*mp).detach()
        if not torch.isfinite(actions).all():raise RuntimeError('non-finite generated actions')
    torch.cuda.synchronize()
    return actions,time.perf_counter()-begun


def experiment(args):
    global AMP_ENABLED,AMP_DTYPE
    out=Path(args.out);out.mkdir(parents=True,exist_ok=True)
    report=dict(scope='fixed-observation offline RTC mechanism and memory test; no robot connection',
        synthetic_schedule=dict(native_hz=30,horizon=30,start_stride=15,assumed_delay=10),
        original_bf16_weights_used=False,compile=False,methods={},errors=[],fixed_seed=args.fixed_seed)
    runtime=ModelRuntime();model=runtime.model;head=model.action_model
    captured={};original_predict=head.predict_action
    def capture(vl,state=None,**kwargs):
        captured['context']=(vl,state,kwargs['action_mask'],kwargs.get('encoder_attention_mask'),kwargs.get('body_type_ids'))
        captured['amp_enabled']=torch.is_autocast_enabled('cuda')
        captured['amp_dtype']=torch.get_autocast_dtype('cuda')
        return original_predict(vl,state,**kwargs)
    head.predict_action=capture
    official,full_elapsed=runtime.predict(runtime.examples[0],10000)
    head.predict_action=original_predict
    # Tensors made under inference_mode cannot be saved by autograd; clone outside.
    ctx=tuple(v.detach().clone()if v is not None else None for v in captured['context'])
    AMP_ENABLED=captured['amp_enabled'];AMP_DTYPE=captured['amp_dtype']
    report['original_autocast']=dict(enabled=AMP_ENABLED,dtype=str(AMP_DTYPE))
    del captured,original_predict
    for p in model.parameters():p.requires_grad_(False)
    torch.cuda.empty_cache();report['full_w8a8_memory']=memory()
    report['full_w8a8_inference_s']=full_elapsed
    report['context_shape']=list(ctx[0].shape)
    reference,se=sample(head,ctx,10000)
    error=float(np.max(np.abs(reference.cpu().numpy()-official)))
    report['sampler_vs_existing_max_abs']=error
    if error>1e-4:raise RuntimeError(f'sampler mismatch {error}')
    arrays={};residency='all_components_gpu'

    def run_method(name,beta):
        nonlocal residency
        gc.collect();torch.cuda.empty_cache();torch.cuda.reset_peak_memory_stats()
        chunks=[];seconds=[];previous=None
        for i in range(args.chunks):
            prior=None
            if previous is not None:
                prior=torch.cat([previous[:,15:],torch.zeros_like(previous[:,:15])],dim=1)
            value,elapsed=sample(head,ctx,10000 if args.fixed_seed else 10000+i,prior=prior,beta=beta)
            chunks.append(value.cpu().numpy());seconds.append(elapsed);previous=value
            print('CHUNK',name,i,round(elapsed,3),flush=True)
        arrays[name]=np.concatenate(chunks)
        report['methods'][name]=dict(beta=beta,head_seconds=seconds,memory=memory(),residency=residency,
            latency_scope='cached conditioning, action head only, CUDA synchronized; encoder and transport excluded')
        np.savez(out/'normalized-chunks.npz',**arrays)
        (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')

    run_method('w8a8_off',0.)
    W8A8Linear.forward=ste_forward
    try:run_method('w8a8_ste_rtc_beta1',1.)
    except torch.cuda.OutOfMemoryError:
        report['errors'].append('STE RTC OOM with all components resident')
        model.qwen_vl_interface.to('cpu');gc.collect();torch.cuda.empty_cache();residency='vlm_cpu_cached_context'
        run_method('w8a8_ste_rtc_beta1',1.)
    finally:W8A8Linear.forward=ORIGINAL_LINEAR
    if args.only_int8:
        np.save(out/'state-anchor.npy',runtime.examples[0]['state'])
        report.update(instruction=runtime.examples[0]['lang'],complete=True)
        (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
        print('EXPERIMENT_COMPLETE',json.dumps(report),flush=True)
        return
    # Replace the head in place; this is not a second model or original BF16.
    runtime.integer_modules={n:m for n,m in runtime.integer_modules.items()if not n.startswith('action_model.')}
    try:report['dequantized_head_modules']=dequantize_head(head)
    except torch.cuda.OutOfMemoryError:
        report['errors'].append('head conversion OOM; VLM moved to CPU')
        model.qwen_vl_interface.to('cpu');gc.collect();torch.cuda.empty_cache();residency='vlm_cpu_cached_context'
        report['dequantized_head_modules_remaining']=dequantize_head(head)
    for name,beta in [('dequant_head_off',0.),('dequant_head_rtc_beta1',1.),('dequant_head_rtc_beta5',5.)]:
        try:run_method(name,beta)
        except torch.cuda.OutOfMemoryError:
            report['errors'].append(name+' OOM; VLM moved to CPU')
            model.qwen_vl_interface.to('cpu');gc.collect();torch.cuda.empty_cache();residency='vlm_cpu_cached_context'
            run_method(name,beta)
    np.save(out/'state-anchor.npy',runtime.examples[0]['state'])
    report['instruction']=runtime.examples[0]['lang']
    report['complete']=True
    (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print('EXPERIMENT_COMPLETE',json.dumps(report),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out',required=True);parser.add_argument('--chunks',type=int,default=4)
    parser.add_argument('--fixed-seed',action='store_true');parser.add_argument('--only-int8',action='store_true')
    experiment(parser.parse_args())
