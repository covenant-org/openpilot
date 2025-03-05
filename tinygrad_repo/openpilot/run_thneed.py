#!/usr/bin/env python3
import os
import sys
import numpy as np
import pyopencl as cl
import io
import cv2
from pathlib import Path
import onnx
from extra.utils import fetch
from extra.thneed import Thneed
from tinygrad.tensor import Tensor
from tinygrad.helpers import dtypes, getenv
from tinygrad.runtime.ops_gpu import CL

thneed_path = '/data/output.thneed'
NUCLEA_MODEL = "https://github.com/covenant-org/tinygrad/releases/download/yoloV8-Medium-NucleaV9/best.onnx"

class ThneedRunner:
    def __init__(self, thneed_path, onnx_path):
        self.thneed = self.load_thneed(thneed_path)
        self.input_shapes = self.get_input_shapes(onnx_path)

    def load_thneed(self, thneed_path):
        nt = Thneed()
        nt.load(thneed_path)
        return nt

    def get_input_shapes(self, onnx_path):
        onnx_model = onnx.load(onnx_path)
        input_shapes = {inp.name: tuple(x.dim_value for x in inp.type.tensor_type.shape.dim) for inp in onnx_model.graph.input}
        return input_shapes

    def preprocess_image(self, image, target_size):
        resized_image = cv2.resize(image, target_size)
        normalized_image = resized_image.astype(np.float32) / 255.0
        chw_image = np.transpose(normalized_image, (2, 0, 1))
        batched_image = np.expand_dims(chw_image, axis=0)

        new_inputs_numpy = {"images": batched_image}

        for k, v in new_inputs_numpy.items():
            print(f"{k}: {v.shape}")

        inputs = {k: Tensor(v).realize() for k, v in new_inputs_numpy.items()}
        return inputs

    def prepare_inputs(self):
        target_size = tuple(reversed(self.input_shapes['images'][2:]))
        gt_path = Path("/data/tinygrad/models/ground-truth")
        images_sample = sorted([img for ext in ["*.jpg", "*.jpeg", "*.png"] for img in gt_path.glob(ext)])

        image = cv2.imread(images_sample[1])
        inputs = self.preprocess_image(image, target_size)
        new_np_inputs = {k: v.realize().numpy() for k, v in inputs.items()}
        return inputs, new_np_inputs

    def execute_thneed(self, new_np_inputs):
        # Cargar las entradas en los buffers de entrada de Thneed
        for k, v in self.thneed.inputs.items():
            cl.enqueue_copy(CL.cl_queue[0], v, new_np_inputs[k], is_blocking=True)

        # Ejecutar el plan de ejecución de Thneed
        self.thneed.run()

        # Obtener las salidas del modelo
        new_thneed_out = np.empty((self.thneed.outputs[0].size // 4,), dtype=np.float32).reshape(self.input_shapes['images'])
        cl.enqueue_copy(CL.cl_queue[0], new_thneed_out, self.thneed.outputs[0], is_blocking=True)
        return new_thneed_out

    def run(self):
        # Preparar las entradas del modelo
        inputs, new_np_inputs = self.prepare_inputs()

        # Ejecutar el modelo .thneed
        output = self.execute_thneed(new_np_inputs)

        # Mostrar las salidas
        print("Output:", output)

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python run_thneed.py <path_to_thneed_file> <path_to_onnx_file>")
        sys.exit(1)
        
    thneed_path = Path(sys.argv[1])
    onnx_path = Path(sys.argv[2])     
    if not onnx_path.is_file:
        onnx_data = fetch(NUCLEA_MODEL)
        onnx_path = io.BytesIO(onnx_data)
    
    runner = ThneedRunner(thneed_path, onnx_path)
    runner.run()