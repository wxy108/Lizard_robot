"""Robot-independent RFT bindings and force provider (legacy remains separate)."""

__all__ = ["Surface", "bind_surfaces", "load_model", "load_surfaces", "RFTForceProvider"]


def __getattr__(name):
    # Importing rft.kernel must not load a simulator or mesh-processing stack.
    if name in __all__:
        from importlib import import_module

        module = ".provider" if name == "RFTForceProvider" else ".binding"
        return getattr(import_module(module, __name__), name)
    raise AttributeError(name)
