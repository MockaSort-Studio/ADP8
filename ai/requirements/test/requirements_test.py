import sys


def main() -> int:
    success = True
    try:
        print("Importing torch/torchvision/lightning...")
        import torch
        import lightning
        import torchvision
        import torchaudio
        import torchmetrics
        import tensorboard
        import comet_ml
        import datasets
        import transformers
        import gymnasium

        print("Import successful!")
    except ImportError as e:
        print(f"Import failed: {e}")
        success = False

    try:
        print("Importing numpy/pandas/matplotlib...")
        import numpy
        import pandas
        import matplotlib
        import jsonargparse
        import mlxtend

        print("Import successful!")
    except ImportError as e:
        print(f"Import failed: {e}")
        success = False

    return 0 if success else -1


if __name__ == "__main__":
    sys.exit(main())
