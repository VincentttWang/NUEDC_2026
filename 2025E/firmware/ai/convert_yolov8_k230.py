"""Convert the 224x224 target detector ONNX model to a K230 KModel."""

import argparse
import os
from pathlib import Path
import shutil
import sysconfig

# nncase 2.x discovers its native assemblies from the Python package directory.
site_packages = sysconfig.get_paths()["purelib"]
os.environ["PATH"] = site_packages + os.pathsep + os.environ.get("PATH", "")

import nncase
import numpy as np
import onnx
import onnxsim
from PIL import Image


INPUT_SHAPE = [1, 3, 224, 224]
IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp"}


def simplify_onnx(source: Path, destination: Path) -> Path:
    model = onnx.load(str(source))
    model = onnx.shape_inference.infer_shapes(model)
    input_name = model.graph.input[0].name
    simplified, valid = onnxsim.simplify(
        model, input_shapes={input_name: INPUT_SHAPE})
    if not valid:
        raise RuntimeError("onnxsim could not validate the simplified model")
    onnx.save(simplified, str(destination))
    return destination


def calibration_data(directory: Path, count: int):
    paths = sorted(
        path for path in directory.iterdir()
        if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES)
    if len(paths) < count:
        raise ValueError(
            f"need {count} calibration images, found {len(paths)} in {directory}")

    # Spread samples through the directory instead of taking one short burst.
    indexes = np.linspace(0, len(paths) - 1, count, dtype=int)
    samples = []
    for index in indexes:
        with Image.open(paths[int(index)]) as image:
            image = image.convert("RGB").resize((224, 224), Image.Resampling.BILINEAR)
            tensor = np.asarray(image, dtype=np.uint8).transpose(2, 0, 1)
            samples.append([tensor[np.newaxis, ...]])
    return np.asarray(samples)


def convert(model_path: Path, dataset_path: Path, output_path: Path, count: int):
    dump_dir = output_path.parent / "nncase_dump"
    if dump_dir.exists():
        shutil.rmtree(dump_dir)
    dump_dir.mkdir(parents=True)

    simplified_path = simplify_onnx(model_path, dump_dir / "simplified.onnx")

    options = nncase.CompileOptions()
    options.target = "k230"
    options.preprocess = True
    options.swapRB = False
    options.input_shape = INPUT_SHAPE
    options.input_type = "uint8"
    options.input_range = [0, 1]
    options.mean = [0, 0, 0]
    options.std = [1, 1, 1]
    options.input_layout = "NCHW"
    options.quant_type = "uint8"

    compiler = nncase.Compiler(options)
    compiler.import_onnx(simplified_path.read_bytes(), nncase.ImportOptions())

    ptq = nncase.PTQTensorOptions()
    ptq.samples_count = count
    ptq.calibrate_method = "NoClip"
    ptq.quant_type = "uint8"
    ptq.w_quant_type = "int16"
    ptq.set_tensor_data(calibration_data(dataset_path, count))
    compiler.use_ptq(ptq)

    compiler.compile()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_bytes(compiler.gencode_tobytes())
    shutil.rmtree(dump_dir)
    print(f"wrote {output_path} ({output_path.stat().st_size} bytes)")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--calibration-count", type=int, default=20)
    args = parser.parse_args()
    convert(args.model, args.dataset, args.output, args.calibration_count)


if __name__ == "__main__":
    main()
