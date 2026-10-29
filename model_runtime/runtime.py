"""Local runtime for the original C-W8A8 bundle."""
import json
import os
from datetime import datetime
from pathlib import Path
import pickle
import sys
import threading
import time
from zoneinfo import ZoneInfo

os.environ.setdefault("HF_HUB_OFFLINE", "1")
os.environ.setdefault("TRANSFORMERS_OFFLINE", "1")
os.environ.setdefault("TOKENIZERS_PARALLELISM", "false")

import numpy as np
import torch

ROOT = Path(__file__).resolve().parent
BUNDLE = Path(os.environ.get("WMA_MODEL_BUNDLE", "/home/wla/WMA-JLee/models/C-W8A8-20261028"))


class ModelRuntime:
    def __init__(self):
        sys.path.insert(0, str(BUNDLE))
        from native_bundle import load_native
        from native_int8 import W8A8Linear

        torch.set_num_threads(4)
        torch.backends.cuda.matmul.allow_tf32 = False
        self.lock = threading.Lock()
        started = time.perf_counter()
        self.model = load_native(BUNDLE)
        torch.cuda.synchronize()
        self.load_seconds = time.perf_counter() - started
        self.logits_to_keep = int(os.environ.get("WMA_LOGITS_TO_KEEP", "1"))
        if self.logits_to_keep not in (0, 1):
            raise ValueError("WMA_LOGITS_TO_KEEP must be 0 (original) or 1 (last token)")
        if self.logits_to_keep:
            # predict_action consumes all hidden states and discards logits.
            # The supported HF argument slices only the unused vocabulary head,
            # after the backbone has produced the complete hidden-state output.
            def limit_unused_logits(module, args, kwargs):
                kwargs.setdefault("logits_to_keep", self.logits_to_keep)
                return args, kwargs

            self._logits_hook = self.model.qwen_vl_interface.model.register_forward_pre_hook(
                limit_unused_logits, with_kwargs=True)
        self.examples = pickle.loads((BUNDLE / "offline-observations.pkl").read_bytes())
        self.reference = np.load(BUNDLE / "native-predictions.npz")["normalized"]
        self.integer_modules = {name: module for name, module in self.model.named_modules()
                                if isinstance(module, W8A8Linear)}
        self.compile_config = {"enabled": False}
        if os.environ.get("WMA_TORCH_COMPILE", "0") == "1":
            from compiled_ops import install
            self.compile_config = install(self.integer_modules)
            print("TORCH_COMPILE_CONFIG", self.compile_config, flush=True)

    def compile_stats(self):
        if not self.compile_config["enabled"]:
            return self.compile_config
        from torch._dynamo.utils import counters
        from torch._inductor import metrics
        return {**self.compile_config, "dynamo_stats": dict(counters["stats"]),
                "graph_breaks": dict(counters["graph_break"]),
                "inductor_generated_kernel_count": metrics.generated_kernel_count}

    def metadata(self):
        import accelerate
        import diffusers
        import transformers

        return {"model": BUNDLE.name, "gpu": torch.cuda.get_device_name(),
                "torch": torch.__version__, "cuda": torch.version.cuda,
                "python": sys.version.split()[0], "transformers": transformers.__version__,
                "accelerate": accelerate.__version__, "diffusers": diffusers.__version__,
                "compute_capability": list(torch.cuda.get_device_capability()),
                "action_horizon": self.model.action_horizon, "action_dim": 54,
                "denoising_steps": self.model.action_model.num_inference_timesteps,
                "language_logits_to_keep": self.logits_to_keep,
                "torch_compile": self.compile_stats(),
                "startup_validation": getattr(self, "startup_validation", []),
                "integer_module_count": len(self.integer_modules),
                "load_seconds": self.load_seconds,
                "gpu_allocated_bytes": torch.cuda.memory_allocated(),
                "gpu_reserved_bytes": torch.cuda.memory_reserved()}

    def predict(self, observation, seed=None, rtc=None):
        if rtc is not None:
            raise ValueError("RTC is excluded from the PAOS model Interface baseline")
        with self.lock, torch.inference_mode():
            if seed is not None:
                torch.manual_seed(seed)
                torch.cuda.manual_seed_all(seed)
            torch.cuda.synchronize()
            started = time.perf_counter()
            if self.compile_config.get("cuda_graphs"):
                torch.compiler.cudagraph_mark_step_begin()
            output = self.model.predict_action([observation])["normalized_actions"]
            torch.cuda.synchronize()
            elapsed = time.perf_counter() - started
            if not np.isfinite(output).all():
                raise RuntimeError("Model returned non-finite actions")
        return output, elapsed

    def benchmark(self, repetitions=32, warmups=2):
        # The API serializes benchmark and prediction requests at the application layer.
        warmup_times = []
        for index in range(warmups):
            _, elapsed = self.predict(self.examples[index % len(self.examples)], 10000 + index % len(self.examples))
            warmup_times.append(elapsed)
            print("BENCHMARK_WARMUP", index, "seconds", round(elapsed, 6), flush=True)
        torch.cuda.reset_peak_memory_stats()
        times, values, indices = [], [], []
        for index in range(repetitions):
            sample = index % len(self.examples)
            output, elapsed = self.predict(self.examples[sample], 10000 + sample)
            values.append(output)
            times.append(elapsed)
            indices.append(sample)
            print("BENCHMARK_SAMPLE", index, "seconds", round(elapsed, 6), flush=True)
        array = np.concatenate(values)
        delta = array - self.reference[indices]
        report = {**self.metadata(), "measured_at": datetime.now(ZoneInfo("Asia/Seoul")).isoformat(),
                  "samples": repetitions, "warmups": warmups, "warmup_seconds": warmup_times,
                  "latency_scope": "batch1 full predict_action, image processor included, four denoising steps, CUDA synchronized; load and HTTP transport excluded",
                  "latency_seconds": times, "latency_mean_s": float(np.mean(times)),
                  "latency_median_s": float(np.median(times)),
                  "latency_p95_s": float(np.percentile(times, 95)),
                  "latency_min_s": min(times), "latency_max_s": max(times),
                  "throughput_chunks_per_s": float(1 / np.mean(times)),
                  "output_shape": list(array.shape), "finite": bool(np.isfinite(array).all()),
                  "cloud_same_quantized_model_rmse": float(np.sqrt(np.mean(delta ** 2))),
                  "cloud_same_quantized_model_max_abs": float(np.abs(delta).max()),
                  "cross_platform_rmse_limit": 0.01,
                  "cross_platform_check_pass": bool(np.sqrt(np.mean(delta ** 2)) < 0.01),
                  "all_integer_modules_executed": all(m.calls > 0 for m in self.integer_modules.values()),
                  "peak_gpu_allocated_bytes": torch.cuda.max_memory_allocated(),
                  "peak_gpu_reserved_bytes": torch.cuda.max_memory_reserved(),
                  "motor_commands_sent": False}
        report_name = "rtx3080-compile-benchmark.json" if self.compile_config["enabled"] else "rtx3080-benchmark.json"
        destination = ROOT / "reports" / report_name
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(json.dumps(report, indent=2))
        np.savez(destination.with_suffix(".npz"), normalized=array, latency=times, sample_indices=indices)
        return report
