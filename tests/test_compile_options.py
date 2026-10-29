import pytest
from unifolm_paos_adapter.compile_options import compile_options


BASE={'triton.cudagraphs':False,'max_autotune':False,
      'max_autotune_gemm_backends':'ATEN','emulate_precision_casts':True}
PRESETS={'max-autotune':{'max_autotune':True,'coordinate_descent_tuning':True,'triton.cudagraphs':True},
         'max-autotune-no-cudagraphs':{'max_autotune':True,'coordinate_descent_tuning':True}}


def test_baseline_unchanged_and_profiles_are_explicit():
    assert compile_options('baseline',BASE,PRESETS)==BASE
    full=compile_options('max-autotune',BASE,PRESETS)
    assert full['max_autotune']and full['coordinate_descent_tuning']and full['triton.cudagraphs']
    assert full['emulate_precision_casts']and full['max_autotune_gemm_backends']=='ATEN'
    lower=compile_options('max-autotune-no-cudagraphs',BASE,PRESETS)
    assert lower['max_autotune']and not lower['triton.cudagraphs']
    assert BASE['max_autotune']is False
    with pytest.raises(ValueError):compile_options('best_effort',BASE,PRESETS)
