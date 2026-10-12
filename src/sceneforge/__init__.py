"""SceneForge public contracts.

Keep package initialization dependency-light.  Control-core modules are used in
minimal runtime environments on Spark and must not import the Pydantic scene
model stack merely because Python initializes the :mod:`sceneforge` package.
The public model names remain available through lazy attribute loading.
"""

from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from .models import AssetRecord, BuildingSpec, SceneSpec

__all__ = ["AssetRecord", "BuildingSpec", "SceneSpec"]
__version__ = "0.1.0"


def __getattr__(name: str) -> Any:
    if name in __all__:
        from . import models

        return getattr(models, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
