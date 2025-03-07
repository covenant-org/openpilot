#!/usr/bin/env python3
import os
import sys
import pickle
import time
import cv2
import onnx
import onnxruntime as ort
import io
import numpy as np
import yaml
import requests
from pathlib import Path
from extra.utils import fetch
import zipfile
import shutil
import argparse
import pyopencl as cl

# Agregar directorio de 'extra'
sys.path.append(str(Path(__file__).resolve().parent.parent))

# Importar módulos de 'extra' y 'tinygrad'
from tinygrad.tensor import Tensor
from tinygrad.helpers import DEBUG
from extra.thneed import Thneed
from tinygrad.runtime.ops_gpu import CL


# Mostrar dispositivos disponibles

# Establecer variables de entorno si no están definidas
os.environ.setdefault("JIT", "2")
os.environ.setdefault("IMAGE", "1")
os.environ.setdefault("FLOAT16", "1")
os.environ.setdefault("DEFAULT_FLOAT", "HALF")
os.environ.setdefault("PREREALIZE", "0")

# os.environ.setdefault("CUDA", "1")
# os.environ.setdefault("DEBUG", "4")
# os.environ.setdefault("NOLOCALS", "1") # REDUCE OPTIMIZACION
# os.environ.setdefault("PTX", "1")
# os.environ.setdefault("PROFILE", "1")
# os.environ.setdefault("JIT_BATCH_SIZE", "0")

# Crear el parser
parser = argparse.ArgumentParser(description="Descargar modelo y dataset")
# Agregar argumentos
parser.add_argument("--url_model", type=str, default="https://github.com/covenant-org/tinygrad/releases/download/yoloV8-Medium-NucleaV9/best.onnx", help="URL del modelo ONNX")
parser.add_argument("--url_dataset", type=str, default="https://app.roboflow.com/ds/qnXOxt8VKv?key=1mmF2G81LD", help="URL del dataset")
parser.add_argument("--thneed_path", type=str, default="", help="Direccion del pkl")
parser.add_argument("--onnx_path", type=str, default="", help="Direccion del pkl")
parser.add_argument("--imshow", type=str, default="True", help="Mostrar imágenes a tiempo real")
args = parser.parse_args()

args.imshow = args.imshow.lower() in ["true", "1", "yes"]

# Usar los valores en el código
print(f"URL del modelo: {args.url_model}")
print(f"URL del dataset: {args.url_dataset}")

thneed_path = '/data/output.thneed'
NUCLEA_MODEL = "https://github.com/covenant-org/tinygrad/releases/download/yoloV8-Medium-NucleaV9/best.onnx"


debug = False

class Timer:
    def __init__(self):
        self.start_time = None
    
    def start(self):
        """Inicia el timer."""
        self.start_time = time.time()
    
    def stop(self):
        """Detiene el timer y devuelve el tiempo transcurrido en segundos."""
        if self.start_time is None:
            raise ValueError("El timer no ha sido iniciado. Usa start() primero.")
        return int(round(((time.time() - self.start_time) * 1000)))

class ModelDownloader:
    def __init__(self, url_model: str = None, url_dataset: str = None):
        self.base_path = Path("/data/tinygrad/models")
        self.url_model = url_model
        if self.url_model:
            self.model_name = self.url_model.split("/")[-2]
        self.model_path_onnx = self.base_path / self.model_name / "best.onnx"
        self.zip_path = self.base_path / "dataset.zip"
        self.gt_path = self.base_path / "ground-truth"
        self.url_dataset = url_dataset
        
        self.base_path.mkdir(parents=True, exist_ok=True)
 
    def download_model(self):
        self.model_path_onnx.parent.mkdir(parents=True, exist_ok=True)

        if not self.model_path_onnx.is_file():
            print(f"El modelo ONNX no existe en: {self.model_path_onnx}")
            print(f"Descargando desde {self.url_model}...")
            
            response = requests.get(self.url_model, allow_redirects=True)
            with open(self.model_path_onnx, "wb") as file:
                file.write(response.content)
            print(f"Modelo descargado en {self.model_path_onnx}")
    
    def download_dataset(self):
        self.gt_path.mkdir(parents=True, exist_ok=True)
        
        if not self.zip_path.is_file():
            print(f"Descargando dataset desde: {self.url_dataset}")
            response = requests.get(self.url_dataset, allow_redirects=True)
            with open(self.zip_path, "wb") as file:
                file.write(response.content)
        if not any(self.gt_path.iterdir()):
            print(f"Descomprimiendo dataset desde {self.zip_path}...")
            with zipfile.ZipFile(self.zip_path, "r") as zip_ref:
                zip_ref.extractall(self.gt_path)
            print(f"Descomprimido en: {self.gt_path}")
            self.merge_folders(self.gt_path)
        self.get_yaml_data()
        self.get_dataset_images()
        
    def get_yaml_data(self):
        self.yaml_path = self.gt_path / Path("data.yaml")
        # Cargar clases desde el archivo YAML
        with self.yaml_path.open("r") as f:
            data = yaml.safe_load(f)
        self.classes = data.get("names", [])
        np.random.seed(42)
        self.classes_color = np.random.uniform(0, 255, size=(len(self.classes), 3)) # Generar colores para cada clase

    def merge_folders(self, gt_path: Path):
        for subdir in list(gt_path.iterdir()):  
            if subdir.is_dir() and subdir != gt_path:  # Asegurar que es una subcarpeta
                # print(f"Fusionando: {subdir}")
                for content in list(subdir.iterdir()):
                    dest_path = gt_path
                    if content.is_dir():  # Fusionar carpetas
                        for sub_content in content.iterdir():
                            shutil.move(str(sub_content), str(dest_path / sub_content.name))
                        content.rmdir()
                    else:
                        shutil.move(str(content), str(dest_path))
                subdir.rmdir()
        # print(f"¡Carpetas fusionadas en '{gt_path}'!")
    
    def get_dataset_images(self):
      self.images_sample = sorted([img for ext in ["*.jpg", "*.jpeg", "*.png"] for img in self.gt_path.glob(ext)])
      # image_path = dir_images_path / Path("Avances-Elite-Toluquilla-II-a-julio-2024_mp4-0021.jpg") # Imagen especifica
      return self.images_sample

class ThneedRunner:
    def __init__(self, thneed_path, onnx_path, model_files):
        self.thneed = self.load_thneed(thneed_path)
        self.input_shapes, self.output_shapes = self.get_onnx_shapes(onnx_path)
        self.model_files = model_files
        
        self.acum_time = []

    def load_thneed(self, thneed_path):
        nt = Thneed()
        nt.load(thneed_path)
        return nt

    def get_onnx_shapes(self, onnx_path):
        onnx_model = onnx.load(onnx_path)
        input_shapes = {inp.name: tuple(x.dim_value for x in inp.type.tensor_type.shape.dim) for inp in onnx_model.graph.input}
        output_shapes = {out.name: tuple(x.dim_value for x in out.type.tensor_type.shape.dim) for out in onnx_model.graph.output}
        return input_shapes, output_shapes['output0']

    def preprocess_image(self, image, target_size):
        resized_image = cv2.resize(image, target_size)
        normalized_image = resized_image.astype(np.float32) / 255.0
        chw_image = np.transpose(normalized_image, (2, 0, 1))
        batched_image = np.expand_dims(chw_image, axis=0)

        new_inputs_numpy = {"images": batched_image}

        inputs = {k: Tensor(v).realize() for k, v in new_inputs_numpy.items()}
        return inputs

    def prepare_inputs(self, image):
        target_size = tuple(reversed(self.input_shapes['images'][2:]))
        
        inputs = self.preprocess_image(image, target_size)
        new_np_inputs = {k: v.realize().numpy() for k, v in inputs.items()}
        return inputs, new_np_inputs
    
    def postprocess_image(self, output_tensor, original_image):
        timer = Timer()
        timer.start()

        height, width, _ = original_image.shape
        scale = max(height, width) / 640
        score_threshold = 0.45

        resized_image = cv2.resize(original_image, (640, 480))
        fase1_timer = timer.stop()

        timer.start()
        output_tensor = np.transpose(output_tensor, (0, 2, 1))
        rows = output_tensor.shape[1]

        boxes, scores, class_ids = [], [], []
        for i in range(rows):
            classes_scores = output_tensor[0][i][4:]
            (_, maxScore, _, (_, maxClassIndex)) = cv2.minMaxLoc(classes_scores)
            if maxScore >= score_threshold:
                box = [
                    output_tensor[0][i][0] - 0.5 * output_tensor[0][i][2],
                    output_tensor[0][i][1] - 0.5 * output_tensor[0][i][3],
                    output_tensor[0][i][2],
                    output_tensor[0][i][3]
                ]
                boxes.append(box)
                scores.append(maxScore)
                class_ids.append(maxClassIndex)

        nms_boxes = cv2.dnn.NMSBoxes(boxes, scores, score_threshold, 0.45, 0.5)
        detections = []

        for i in range(len(nms_boxes)):
            index = nms_boxes[i]
            box = boxes[index]
            detection = {
                'class_id': class_ids[index],
                'class_name': self.model_files.classes[class_ids[index]],
                'confidence': scores[index],
                'box': box,
                'scale': scale
            }
            detections.append(detection)

            x, y, x_plus_w, y_plus_h = (
                round(box[0] * scale),
                round(box[1] * scale),
                round((box[0] + box[2]) * scale),
                round((box[1] + box[3]) * scale)
            )
            self.draw_bounding_box(resized_image, class_ids[index], scores[index], x, y, x_plus_w, y_plus_h)

        fase3_timer = timer.stop()

        print(f"Post-Inferencia: fase 1: {fase1_timer}ms, fase 2: {fase3_timer}ms")
        return resized_image

    def execute_thneed(self, new_np_inputs):
        # Cargar las entradas en los buffers de entrada de Thneed
        for k, v in self.thneed.inputs.items():
            cl.enqueue_copy(CL.cl_queue[0], v, new_np_inputs[k], is_blocking=True)

        # Ejecutar el plan de ejecución de Thneed
        self.thneed.run()

        # Obtener las salidas del modelo
        new_thneed_out = np.empty((self.thneed.outputs[0].size // 4,), dtype=np.float32).reshape(self.output_shapes)
        cl.enqueue_copy(CL.cl_queue[0], new_thneed_out, self.thneed.outputs[0], is_blocking=True)
        return new_thneed_out

    def run(self, image):
        timer = Timer()
        timer.start()
        # Preparar las entradas del modelo
        inputs, new_np_inputs = self.prepare_inputs(image)
        preinferencia_time = timer.stop()

        # Ejecutar el modelo .thneed
        timer.start()
        output = self.execute_thneed(new_np_inputs)
        inferencia_time = timer.stop()

        timer.start()
        postprocessed_image = self.postprocess_image(output, image)
        postinferencia_time = timer.stop()
        
        total_time = preinferencia_time + inferencia_time + postinferencia_time
        self.acum_time.append(total_time)
        
        return postprocessed_image
    
    def draw_bounding_box(self, img, class_id, confidence, x, y, x_plus_w, y_plus_h):
        label = f'{self.model_files.classes[class_id]} ({confidence:.2f})'
        color = self.model_files.classes_color[class_id]
        cv2.rectangle(img, (x, y), (x_plus_w, y_plus_h), color, 2)
        cv2.putText(img, label, (x - 10, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

def main():
    os.chdir("/tmp")
    thneed_path = Path(args.thneed_path)
    onnx_path = Path(args.onnx_path)     
    if not onnx_path.is_file:
        onnx_data = fetch(NUCLEA_MODEL)
        onnx_path = io.BytesIO(onnx_data)
    
    model_files = ModelDownloader(args.url_model, args.url_dataset)
    model_files.download_model()
    model_files.download_dataset()
    
    images_sample = [cv2.imread(img) for img in model_files.images_sample[:3]]

    # tinygrad_model = ModelProcessor(model_files)
    # tinygrad_model.load_onnx(model_files.model_path_onnx) 
    # tinygrad_model.load_pkl(args.model_path_pkl, images_sample)
    runner = ThneedRunner(thneed_path, onnx_path, model_files)
    

    for image_path in model_files.images_sample:
        original_image = cv2.imread(image_path)
        if original_image is None:
            raise FileNotFoundError(f"No se pudo cargar la imagen \"{image_path}\".")

        predict_image = runner.run(original_image)

        # Mostrar la imagen con las detecciones
    #     if args.imshow:
    #         cv2.imshow("Tinygrad model", predict_image)
    #         if cv2.waitKey(1) & 0xFF == ord('q'):
    #             break
    # cv2.destroyAllWindows()
    print(f"\tMedian: {np.median(runner.acum_time)}ms")

    # # Guardar o mostrar la imagen con las detecciones
    # output_path = "output_image.jpg"
    # cv2.imwrite(output_path, output_image)
    # print(f"Imagen procesada guardada en: {output_path}")

if __name__ == "__main__":
    main()