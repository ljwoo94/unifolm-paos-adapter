"""Compile the repeated W8A8 tensor operations with TorchInductor.

Keep instrumentation outside the compiled function so per-module counters do not
invalidate guards. The original checkpoint and bundle sources stay unchanged.
"""
from types import MethodType
import os

import torch
import torch.nn.functional as F


def int8_linear(x, qweight_t, scales, pre_scale, bias, out_features):
    shape = x.shape
    in_features = shape[-1]
    kpad = qweight_t.shape[0]
    v = F.pad(x.reshape(-1, in_features) * pre_scale, (0, kpad - in_features))
    scale = v.float().abs().amax(-1, keepdim=True).clamp_min(1e-20) / 127
    q = (v.float() / scale).round().clamp(-127, 127).to(torch.int8).contiguous()
    s = scale.squeeze(-1).contiguous()
    m = q.shape[0]
    mp = max(32, ((m + 7) // 8) * 8)
    q = F.pad(q, (0, 0, 0, mp - m))
    acc = torch._int_mm(q, qweight_t)
    y = (acc[:m].float() * s[:, None] * scales[None, :])[:, :out_features].to(x.dtype)
    if bias is not None:
        y = y + bias
    return y.reshape(*shape[:-1], out_features)


def prepare(x, pre_scale, kpad):
    return F.pad(x.reshape(-1, x.shape[-1]) * pre_scale, (0, kpad - x.shape[-1]))


def finish(acc, activation_scales, weight_scales, bias, x, out_features):
    m = activation_scales.shape[0]
    y = (acc[:m].float() * activation_scales[:, None] * weight_scales[None, :])[:, :out_features].to(x.dtype)
    if bias is not None:
        y = y + bias
    return y.reshape(*x.shape[:-1], out_features)


def install(modules):
    import torch._dynamo.config
    import torch._inductor.config

    torch._dynamo.config.suppress_errors = False
    torch._dynamo.config.recompile_limit = 64
    torch._inductor.config.compile_threads = 4
    os.environ["TRITON_DEFAULT_FP_FUSION"] = "0"
    import triton
    triton.knobs.language.default_fp_fusion = False
    from unifolm_paos_adapter.compile_options import compile_options
    profile = os.environ.get("WMA_COMPILE_PROFILE", "baseline")
    baseline = {"triton.cudagraphs": False, "max_autotune": False,
                "max_autotune_gemm_backends": "ATEN", "emulate_precision_casts": True}
    options = compile_options(profile, baseline, torch._inductor.list_mode_options())
    compiled_prepare = torch.compile(prepare, backend="inductor", dynamic=True, fullgraph=True, options=options)
    compiled_finish = torch.compile(finish, backend="inductor", dynamic=True, fullgraph=True, options=options)
    from native_int8 import quantize_activation

    def forward(self, x):
        self.calls += 1
        v = compiled_prepare(x, self.pre_scale, self.kpad)
        # Keep the division/rounding boundary in the original CUDA quantizer.
        # Approximate division in a fused kernel can change INT8 codes by one.
        q, s = quantize_activation(v)
        m = q.shape[0]
        mp = max(32, ((m + 7) // 8) * 8)
        q = F.pad(q, (0, 0, 0, mp - m))
        acc = torch._int_mm(q, self.qweight_t)
        return compiled_finish(acc, s, self.scales, self.bias, x, self.out_features)

    for module in modules.values():
        module.forward = MethodType(forward, module)
    return {"enabled": True, "backend": "inductor", "scope": "all W8A8Linear preparation and output dequantization",
            "activation_quantizer": "original eager CUDA to preserve exact division and rounding",
            "int8_gemm": "original torch._int_mm / cuBLAS",
            "modules": len(modules), "dynamic": True, "fullgraph": True,
            "profile": profile, "effective_options": options,
            "cuda_graphs": options["triton.cudagraphs"], "max_autotune": options["max_autotune"],
            "emulate_precision_casts": options["emulate_precision_casts"],
            "triton_fp_fusion": False,
            "compile_threads": 4, "recompile_limit": 64}
