#!/usr/bin/env python3
import argparse
import os
import sys

def main():
    parser = argparse.ArgumentParser(description="Export YOLO26 model to ONNX format")
    parser.add_argument("--model", type=str, default="yolo26n.pt", help="YOLO26 model name, e.g., yolo26n.pt, yolo26s.pt")
    parser.add_argument("--output-dir", type=str, default="models", help="Directory to save the exported ONNX model")
    args = parser.parse_args()

    # Determine absolute paths
    script_dir = os.path.dirname(os.path.abspath(__file__))
    pkg_dir = os.path.dirname(script_dir)
    target_dir = os.path.abspath(os.path.join(pkg_dir, args.output_dir))

    try:
        from ultralytics import YOLO
    except ImportError:
        print("Error: 'ultralytics' package is not installed.")
        print("Please install it using: pip install ultralytics")
        sys.exit(1)

    print(f"Loading pretrained model: {args.model}...")
    model = YOLO(args.model)

    print("Exporting model to ONNX format (imgsz=640)...")
    # Export to ONNX. We keep dynamic=False and simplify=True for optimal CPU performance in OpenCV DNN.
    onnx_path = model.export(format="onnx", imgsz=640, dynamic=False, simplify=True)
    
    if onnx_path and os.path.exists(onnx_path):
        os.makedirs(target_dir, exist_ok=True)
        dest = os.path.join(target_dir, os.path.basename(onnx_path))
        os.rename(onnx_path, dest)
        print(f"\nSuccess! Exported model moved to: {dest}")
    else:
        print("Error: Export failed or ONNX file not found.")

if __name__ == "__main__":
    main()
