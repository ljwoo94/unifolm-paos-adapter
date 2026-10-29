"""Explicit TorchInductor profiles; the original quantizer/GEMM stay eager."""
PROFILES=('baseline','max-autotune','max-autotune-no-cudagraphs')


def compile_options(profile,baseline,presets):
    if profile not in PROFILES:raise ValueError(f'unknown compile profile: {profile}')
    result=dict(baseline)
    if profile!='baseline':
        if profile not in presets:raise ValueError(f'installed Torch has no profile {profile}')
        result.update(dict(presets[profile]))
        # Preserve the existing numerical boundary and candidate backend choice.
        for name in ['emulate_precision_casts','max_autotune_gemm_backends']:
            result[name]=baseline[name]
    return result
