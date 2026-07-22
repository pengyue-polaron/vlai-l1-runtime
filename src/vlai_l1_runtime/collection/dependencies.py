"""Single optional-environment boundary for LeRobot dataset operations."""

from __future__ import annotations

import sys


def require_collection_python() -> None:
    if sys.version_info < (3, 12):
        raise RuntimeError(
            "LeRobot collection operations require a Python 3.12 environment "
            "with 'vlai-l1-runtime[dataset]' installed"
        )


def collection_dependency_error() -> RuntimeError:
    return RuntimeError(
        "collection dependencies are missing; install "
        "'vlai-l1-runtime[dataset]' in a Python 3.12 environment"
    )
