import torch

import lightning as L
from typing import Any, Callable, Dict, List, Type


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
    """
    @file model.py
    @brief Contains the implementation of the `lightning_model` decorator for wrapping a PyTorch model into a PyTorch LightningModule.

    @fn lightning_model(cls: Type[torch.nn.Module]) -> Type[L.LightningModule]
    @brief A decorator to encapsulate a PyTorch model class into a PyTorch LightningModule.

    @param cls The class to be wrapped. Must be a subclass of `torch.nn.Module` and implement the required methods: `forward`, `training_step`, and `configure_optimizers`.

    @exception TypeError Raised if the decorated class is not a subclass of `torch.nn.Module`.
    @exception TypeError Raised if the decorated class does not have a callable `forward` method.
    @exception TypeError Raised if the decorated class does not have a callable `training_step` method.
    @exception TypeError Raised if the decorated class does not have a callable `configure_optimizers` method.

    @returns A new class that inherits from `L.LightningModule` and wraps the functionality of the provided class.

    @class LightningWrapper
    @brief A dynamically created wrapper class that extends `L.LightningModule` and encapsulates the provided PyTorch model class.

    @var LightningWrapper.marked_as_forward
    A list of method names in the wrapped class that are marked as forward variants.

    @fn LightningWrapper.__init__(self, *args: Dict[str, Any], **kwargs: Dict[str, Any]) -> None
    @brief Initializes the LightningWrapper by instantiating the wrapped class and exposing marked methods.

    @param args Positional arguments to pass to the wrapped class's constructor.
    @param kwargs Keyword arguments to pass to the wrapped class's constructor.

    @fn LightningWrapper.forward(self, *args: Dict[str, Any], **kwargs: Dict[str, Any]) -> tuple
    @brief Implements the forward method required by `L.LightningModule`. Delegates to the wrapped class's `forward` method.

    @param args Positional arguments to pass to the wrapped class's `forward` method.
    @param kwargs Keyword arguments to pass to the wrapped class's `forward` method.

    @returns The output of the wrapped class's `forward` method.

    @fn LightningWrapper.training_step(self, *args: Dict[str, Any], **kwargs: Dict[str, Any]) -> tuple
    @brief Implements the training_step method required by `L.LightningModule`. Delegates to the wrapped class's `training_step` method.

    @param args Positional arguments to pass to the wrapped class's `training_step` method.
    @param kwargs Keyword arguments to pass to the wrapped class's `training_step` method.

    @returns The output of the wrapped class's `training_step` method.

    @fn LightningWrapper.configure_optimizers(self, lr: float) -> torch.optim.Optimizer
    @brief Implements the configure_optimizers method required by `L.LightningModule`. Delegates to the wrapped class's `configure_optimizers` method.

    @param lr The learning rate to be passed to the wrapped class's `configure_optimizers` method.

    @returns The optimizer configured by the wrapped class's `configure_optimizers` method.

    @note This decorator is useful for integrating PyTorch models into the PyTorch Lightning framework while maintaining flexibility and modularity.

    @remark Documented with precision and care by GitHub Copilot. You're welcome!
    """
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

        def __init__(self, *args: Dict[str, Any], **kwargs: Dict[str, Any]) -> None:
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

        # return values to be revisited
        def forward(self, *args: Dict[str, Any], **kwargs: Dict[str, Any]) -> tuple:
            # Forward method must respect the base interface
            return self.model.forward(*args, **kwargs)

        def training_step(
            self, *args: Dict[str, Any], **kwargs: Dict[str, Any]
        ) -> tuple:
            # Call the wrapped class's training step
            return self.model.training_step(*args, **kwargs)

        def configure_optimizers(self, lr: float) -> torch.optim.Optimizer:
            return self.model.configure_optimizers(lr)

    return LightningWrapper
