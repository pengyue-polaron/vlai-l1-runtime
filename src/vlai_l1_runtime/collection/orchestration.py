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

    with sink:
        frames = write_episode_frames(
            samples=samples,
            assembler=assembler,
            sink=sink,
            task=task,
        )
        return complete_episode(sink=sink, frame_count=frames, decision=decision)


def write_episode_frames(
    *,
    samples: Iterable[tuple[CollectionSample, int]],
    assembler: SampleAssembler,
    sink: EpisodeSink,
    task: str,
) -> int:
    """Validate and append one finite sample stream to an already-open sink."""

    frames = 0
    for sample, now_ns in samples:
        sink.add_frame(assembler.validate(sample, now_ns=now_ns).lerobot_frame(task=task))
        frames += 1
    if frames == 0:
        raise ValueError("an episode must contain at least one frame")
    return frames


def complete_episode(
    *,
    sink: EpisodeSink,
    frame_count: int,
    decision: EpisodeDecision,
) -> EpisodeResult:
    """Commit or discard frames after live capture has stopped."""

    if not isinstance(decision, EpisodeDecision):
        decision = EpisodeDecision(decision)
    if isinstance(frame_count, bool) or not isinstance(frame_count, int) or frame_count <= 0:
        raise ValueError("frame_count must be a positive integer")
    if decision is EpisodeDecision.SAVE:
        return EpisodeResult(decision, frame_count, sink.commit())
    sink.discard()
    return EpisodeResult(decision, frame_count, None)
