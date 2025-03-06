#!/bin/bash

# Ruta del ejecutable de Python
PYTHON_EXEC="/home/manuelo247/tinygrad/tinygrad_env/bin/python"

# Ruta del script de inferencia
SCRIPT_PATH="/data/openpilot/tinygrad_repo/openpilot/compile2_nuclea.py"

# Lista de modelos a descargar
MODELS=(
    "https://github.com/covenant-org/tinygrad/releases/download/yoloV8-Medium-NucleaV9/best.onnx"
    # "https://github.com/covenant-org/tinygrad/releases/download/yoloV8-Small-NucleaV9/best.onnx"
    # "https://github.com/covenant-org/tinygrad/releases/download/yoloV8-Nano-NucleaV9/best.onnx"
)

for MODEL_URL in "${MODELS[@]}"; do
    echo "Ejecutando modelo: $MODEL_URL"
    ORT=1 PYOPENCL_COMPILER_OUTPUT=1 $PYTHON_EXEC $SCRIPT_PATH "$MODEL_URL"
    if [ $? -ne 0 ]; then
        echo "Error ejecutando el modelo: $MODEL_URL"
        exit 1
    fi
done

echo "Todos los modelos han sido procesados correctamente."