"""Sweep completo en torch puro (CPU o GPU), sin Isaac.

Uso (desde tools/pid_sweep):
  python -m core.sweep_torch --quick          # prueba rapida, pocos entornos
  python -m core.sweep_torch                  # barrido completo de params.yaml
  python -m core.sweep_torch --device cuda    # en GPU
"""

from __future__ import annotations

import argparse

import torch

from .model import TorchBody
from .params import PACKAGE_DIR, load_config
from .sweep import quick_config, run_sweep


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=None, help="ruta a params.yaml (default: el del paquete)")
    ap.add_argument("--out", default=str(PACKAGE_DIR / "results"), help="carpeta de salida")
    ap.add_argument("--device", default="cpu", help="cpu o cuda")
    ap.add_argument("--quick", action="store_true", help="pocos entornos y 3 trayectorias")
    ap.add_argument("--threads", type=int, default=0, help="hilos de torch en CPU (0 = default)")
    args = ap.parse_args(argv)

    if args.threads:
        torch.set_num_threads(args.threads)
    cfg = load_config(args.config)
    if args.quick:
        cfg = quick_config(cfg)
    dt = float(cfg["sim"]["physics_dt_s"])
    run_sweep(cfg, lambda env: TorchBody(env, dt, args.device), device=args.device, out_dir=args.out,
              backend=f"torch-{args.device}")


if __name__ == "__main__":
    main()
