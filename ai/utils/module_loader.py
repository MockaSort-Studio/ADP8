import importlib
import pkgutil
import os


def import_symbol_from_file(import_path: str):
    if not all(part.isidentifier() for part in import_path.split(".")):
        raise ValueError(f"{import_path} is not a valid Python import path")

    *module_path, symbol = import_path.split(".")
    module_name = ".".join(module_path)

    # Import the module
    module = importlib.import_module(module_name)

    # Retrieve the symbol from the module
    if hasattr(module, symbol):
        return getattr(module, symbol)
    else:
        raise AttributeError(f"Symbol '{symbol}' not found in module '{module_name}'")


def import_symbol_from_module(package_name: str, symbol: str):
    if not all(part.isidentifier() for part in package_name.split(".")):
        raise ValueError(f"{package_name} is not a valid Python module path")

    package = importlib.import_module(package_name)
    package_path = package.__path__
    for _, module_name, _ in pkgutil.walk_packages(package_path, package_name + "."):
        module = importlib.import_module(module_name)
        if hasattr(module, symbol):
            return getattr(module, symbol)

    raise AttributeError(f"Function '{symbol}' not found in package '{package_name}'")
