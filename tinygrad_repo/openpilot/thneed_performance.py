#!/usr/bin/env python3
import sys
import pathlib
from pathlib import Path
import io
import argparse
import os
import cv2
import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parents[1]))
from yolov8_onnx_thneed import ThneedRunner, Timer, ModelDownloader
from extra.utils import fetch

# Crear el parser
parser = argparse.ArgumentParser(description="Descargar modelo y dataset")
# Agregar argumentos
parser.add_argument("--url_model", type=str, default="https://github.com/covenant-org/tinygrad/releases/download/yoloV8-Medium-NucleaV9/best.onnx", help="URL del modelo ONNX")
parser.add_argument("--url_dataset", type=str, default="https://app.roboflow.com/ds/qnXOxt8VKv?key=1mmF2G81LD", help="URL del dataset")
parser.add_argument("--thneed_path", type=str, default="", help="Direccion del modelo thneed")
parser.add_argument("--onnx_path", type=str, default="", help="Direccion del modelo onnx")
parser.add_argument("--imshow", type=str, default="True", help="Mostrar imágenes a tiempo real")
args = parser.parse_args()

args.imshow = args.imshow.lower() in ["true", "1", "yes"]

# Usar los valores en el código
print(f"URL del modelo: {args.url_model}")
print(f"URL del dataset: {args.url_dataset}")

def main():
    os.chdir("/tmp")
    model_files = ModelDownloader(args.url_model, args.url_dataset)
    model_files.download_model()
    model_files.download_dataset()
    
    # onnx_path = Path(args.onnx_path)     
    # if not onnx_path.is_file:
    onnx_data = fetch(args.url_model)
    onnx_path = io.BytesIO(onnx_data)
    print("ONNX path: ", onnx_path)
        
    thneed_path = Path(args.thneed_path)
    model_name = args.url_model.split("/")[-2]
    # if not thneed_path.is_file and args.url_model:
    #     print(":0")
    thneed_path = model_files.model_path_onnx.parent / (model_name + ".thneed")
    
    print(f"Running model")
    print(f"Loaded in {thneed_path}")
    runner = ThneedRunner(thneed_path, onnx_path, model_files)
    
    for image_path in model_files.images_sample[:10]:
        original_image = cv2.imread(image_path)
        if original_image is None:
            raise FileNotFoundError(f"No se pudo cargar la imagen \"{image_path}\".")

        predict_image = runner.run(original_image)

    print(f"\tMedian: {np.median(runner.acum_time)}ms")

if __name__ == "__main__":
    main()