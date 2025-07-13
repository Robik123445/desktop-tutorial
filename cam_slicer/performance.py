import logging
import os
import time
from concurrent.futures import ProcessPoolExecutor
from typing import Iterable, Callable, List, TypeVar

try:
    import psutil  # type: ignore
except Exception:  # pragma: no cover - optional dep
    psutil = None

T = TypeVar("T")

PERF_THRESHOLD = float(os.getenv("CAM_PERF_THRESHOLD", "0.1"))
MEM_THRESHOLD = int(os.getenv("CAM_MEM_THRESHOLD", "10")) * 1024 * 1024


def profiled(func: Callable[..., T]) -> Callable[..., T]:
    """Measure runtime and memory usage, logging warnings if limits exceeded."""

    def wrapper(*args, **kwargs):
        proc = psutil.Process() if psutil else None
        mem_before = proc.memory_info().rss if proc else 0
        start = time.perf_counter()
        result = func(*args, **kwargs)
        duration = time.perf_counter() - start
        if duration > PERF_THRESHOLD:
            logging.warning("Performance warning: %s took %.3fs", func.__name__, duration)
        else:
            logging.debug("%s runtime %.3fs", func.__name__, duration)
        if proc:
            mem_after = proc.memory_info().rss
            if mem_after - mem_before > MEM_THRESHOLD:
                diff = (mem_after - mem_before) / 1024 / 1024
                logging.warning("Memory usage increased by %.1f MB in %s", diff, func.__name__)
        return result

    return wrapper  # type: ignore


def parallel_map(func: Callable[[T], T], data: Iterable[T], max_workers: int | None = None) -> List[T]:
    """Parallel map helper using processes."""
    with ProcessPoolExecutor(max_workers=max_workers) as ex:
        return list(ex.map(func, data))

