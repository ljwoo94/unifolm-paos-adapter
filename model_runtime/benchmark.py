"""Finite native W8A8 benchmark in a dedicated WLA staging process."""
import argparse
import json
import os
from pathlib import Path


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--profile',choices=['baseline','max-autotune','max-autotune-no-cudagraphs'],default='baseline')
    p.add_argument('--samples',type=int,default=4)
    p.add_argument('--warmups',type=int,default=2)
    a=p.parse_args()
    if not 1<=a.samples<=64 or not 1<=a.warmups<=8:p.error('bounded sample/warmup counts required')
    os.environ['WMA_TORCH_COMPILE']='1';os.environ['WMA_COMPILE_PROFILE']=a.profile
    from runtime import ModelRuntime,ROOT
    runtime=ModelRuntime()
    result=runtime.benchmark(a.samples,a.warmups)
    result['requested_compile_profile']=a.profile
    result['scope']='stored normalized observations, same W8A8 checkpoint; no PAOS/robot commands'
    result.pop('all_chunks_within_threshold',None)
    destination=ROOT/'reports'/f'{a.profile}-benchmark.json'
    destination.write_text(json.dumps(result,indent=2,default=str)+'\n')
    print('BENCHMARK_RESULT',json.dumps({k:result[k]for k in ['model','gpu','requested_compile_profile',
        'latency_mean_s','latency_p95_s','latency_max_s','cloud_same_quantized_model_rmse',
        'peak_gpu_allocated_bytes','peak_gpu_reserved_bytes','warmup_seconds']},default=str),flush=True)
    if not result['finite']or not result['cross_platform_check_pass']:
        raise RuntimeError('native numerical reference check failed')


if __name__=='__main__':main()
