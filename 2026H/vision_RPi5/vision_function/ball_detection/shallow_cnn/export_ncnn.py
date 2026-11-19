"""Export the trained shallow CNN to TorchScript and NCNN with PNNX."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import shutil
import subprocess

import torch

from .evaluate import DEFAULT_WEIGHTS, load_checkpoint


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "model" / "shallow_cnn_v1_ncnn_model"


def find_pnnx(explicit: Path | None) -> Path:
    if explicit is not None:
        if not explicit.is_file():
            raise FileNotFoundError(f"PNNX executable not found: {explicit}")
        return explicit
    discovered = shutil.which("pnnx")
    if discovered is None:
        raise FileNotFoundError(
            "pnnx is not on PATH; pass --pnnx or install requirements-export.txt"
        )
    return Path(discovered)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--weights", type=Path, default=DEFAULT_WEIGHTS)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--pnnx", type=Path)
    args = parser.parse_args()

    pnnx_path = find_pnnx(args.pnnx)
    output_dir = args.output.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    model, checkpoint = load_checkpoint(args.weights, torch.device("cpu"))
    model.eval()
    example = torch.zeros(1, 3, 100, 640, dtype=torch.float32)
    with torch.inference_mode():
        traced = torch.jit.trace(model, example, strict=True)
        reference_outputs = model(example)
    torchscript_path = output_dir / "shallow_cnn.pt"
    traced.save(str(torchscript_path))

    subprocess.run(
        [
            str(pnnx_path),
            str(torchscript_path),
            "inputshape=[1,3,100,640]",
            "fp16=0",
        ],
        cwd=output_dir,
        check=True,
    )
    generated_prefix = output_dir / "shallow_cnn"
    generated_param = generated_prefix.with_suffix(".ncnn.param")
    generated_bin = generated_prefix.with_suffix(".ncnn.bin")
    if not generated_param.is_file() or not generated_bin.is_file():
        raise RuntimeError("PNNX did not generate shallow_cnn.ncnn.param/.bin")
    shutil.copy2(generated_param, output_dir / "model.ncnn.param")
    shutil.copy2(generated_bin, output_dir / "model.ncnn.bin")

    metadata = {
        "schema_version": 1,
        "description": "Shallow steel-ball presence and 1D position model",
        "checkpoint": str(args.weights),
        "checkpoint_epoch": checkpoint.get("epoch"),
        "input_name": "in0",
        "input_shape": [1, 3, 100, 640],
        "input_color": "RGB",
        "input_normalization": "uint8 / 255.0",
        "outputs": [
            {"name": "out0", "description": "presence_logits", "shape": [1]},
            {"name": "out1", "description": "heatmap_logits", "shape": [1, 160]},
            {"name": "out2", "description": "offset_logits", "shape": [1, 160]},
        ],
        "fp16": False,
        "pnnx": subprocess.run(
            [str(pnnx_path), "--version"],
            check=False,
            capture_output=True,
            text=True,
        ).stdout.strip(),
        "reference_output_shapes": [list(output.shape) for output in reference_outputs],
    }
    (output_dir / "metadata.json").write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(metadata, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
