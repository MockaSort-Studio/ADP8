import torch

import lightning as L
from typing import Callable, List, Type


# Fabric has a special way to handle forward functions (i.e. forward pass on the model)
# It does not allow (while training on multiple devices) to have external calls on the model
# sometimes you have a different flavour of forward pass, e.g. for evaluation/inference..
# This decorator marks such functions so that they can be called directly from the LightningModule
# and marked as forward into fabric
# https://lightning.ai/docs/fabric/2.5.1/api/wrappers.html
def forward_variant(func: Callable) -> Callable:
    func.__forward_variant__ = True
    return func


def lightning_model(cls: Type[torch.nn.Module]) -> Type[L.LightningModule]:
    """Decorator to encapsulate a class into a LightningModule."""
    if not issubclass(cls, torch.nn.Module):
        raise TypeError("The decorated class must be a subclass of torch.nn.Module.")
    if not hasattr(cls, "forward") or not callable(getattr(cls, "forward")):
        raise TypeError("The decorated class must have a callable 'forward' method.")
    if not hasattr(cls, "training_step") or not callable(getattr(cls, "training_step")):
        raise TypeError(
            "The decorated class must have a callable 'training_step' method."
        )
    if not hasattr(cls, "configure_optimizers") or not callable(
        getattr(cls, "configure_optimizers")
    ):
        raise TypeError(
            "The decorated class must have a callable 'configure_optimizers' method."
        )

    class LightningWrapper(L.LightningModule):
        marked_as_forward: List[str] = []

        def __init__(self, *args, **kwargs):
            super().__init__()
            print(*args, **kwargs)  # Debugging: print the arguments
            self.model = cls(*args, **kwargs)  # Instantiate the wrapped class

            # Dynamically expose marked methods
            for name in dir(self.model):
                attr = getattr(self.model, name)
                if callable(attr) and getattr(attr, "__forward_variant__", False):
                    # Expose the marked method in the LightningModule
                    self.marked_as_forward.append(name)
                    setattr(self, name, attr)

        def forward(self, *args, **kwargs):
            # Forward method must respect the base interface
            return self.model.forward(*args, **kwargs)

        def training_step(self, *args, **kwargs):
            # Call the wrapped class's training step
            return self.model.training_step(*args, **kwargs)

        def configure_optimizers(self, lr: float):
            return self.model.configure_optimizers(lr)

    return LightningWrapper
