"""VLAI L1 collection contracts and LeRobot dataset tooling."""

from .configuration import CollectionConfig, load_collection_config
from .dataset import (
    DirectDatasetIdentity,
    DirectDatasetState,
    DirectLeRobotEpisode,
    identity_from_config,
    inspect_direct_dataset,
    provenance_from_config,
)
from .mock import SyntheticSampleSource
from .orchestration import EpisodeResult, record_episode
from .schema import (
    ACTION_KEY,
    AGENT_IMAGE_KEY,
    STATE_KEY,
    WRIST_IMAGE_KEY,
    CameraSample,
    CollectionSample,
    DatasetContract,
    SampleAssembler,
)

__all__ = [
    "ACTION_KEY",
    "AGENT_IMAGE_KEY",
    "STATE_KEY",
    "WRIST_IMAGE_KEY",
    "CameraSample",
    "CollectionConfig",
    "CollectionSample",
    "DatasetContract",
    "DirectDatasetIdentity",
    "DirectDatasetState",
    "DirectLeRobotEpisode",
    "EpisodeResult",
    "SampleAssembler",
    "SyntheticSampleSource",
    "identity_from_config",
    "inspect_direct_dataset",
    "load_collection_config",
    "provenance_from_config",
    "record_episode",
]
