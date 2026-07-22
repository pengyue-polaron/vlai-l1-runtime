"""Hardware-independent episode lifecycle composed with embodied-ops."""

from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol

from embodied_ops import EpisodeDecision

from .schema import CollectionSample, SampleAssembler


class EpisodeSink(Protocol):
    def __enter__(self) -> EpisodeSink: ...

    def add_frame(self, frame: dict[str, object]) -> None: ...

    def commit(self) -> Path: ...

    def discard(self) -> None: ...

    def __exit__(self, exc_type, exc, traceback) -> None: ...


@dataclass(frozen=True)
class EpisodeResult:
    decision: EpisodeDecision
    frame_count: int
    dataset_root: Path | None


def record_episode(
    *,
    samples: Iterable[tuple[CollectionSample, int]],
    assembler: SampleAssembler,
    sink: EpisodeSink,
    task: str,
    decision: EpisodeDecision,
) -> EpisodeResult:
    """Validate and write one finite sample stream, then save or discard explicitly."""

    if not isinstance(decision, EpisodeDecision):
        decision = EpisodeDecision(decision)
    frames = 0
    with sink:
        for sample, now_ns in samples:
            sink.add_frame(assembler.validate(sample, now_ns=now_ns).lerobot_frame(task=task))
            frames += 1
        if frames == 0:
            raise ValueError("an episode must contain at least one frame")
        if decision is EpisodeDecision.SAVE:
            return EpisodeResult(decision, frames, sink.commit())
        sink.discard()
        return EpisodeResult(decision, frames, None)
