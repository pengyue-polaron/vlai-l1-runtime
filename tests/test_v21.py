from __future__ import annotations

from vlai_l1_runtime.collection.v21 import V21_DATA_PATH, V21_VIDEO_PATH, _v21_info


def test_v21_metadata_is_episode_based_and_preserves_feature_names() -> None:
    source = {
        "robot_type": "vlai_l1",
        "fps": 30,
        "total_episodes": 2,
        "total_frames": 12,
        "total_tasks": 1,
        "features": {
            "observation.state": {
                "dtype": "float32",
                "shape": [16],
                "names": [f"joint_{index}" for index in range(16)],
            },
            "observation.images.agent": {
                "dtype": "video",
                "shape": [480, 640, 3],
                "names": ["height", "width", "channel"],
            },
        },
    }
    result = _v21_info(source, video_keys=["observation.images.agent"])

    assert result["codebase_version"] == "v2.1"
    assert result["data_path"] == V21_DATA_PATH
    assert result["video_path"] == V21_VIDEO_PATH
    assert (
        result["features"]["observation.state"]["names"]
        == source["features"]["observation.state"]["names"]
    )
    assert result["features"]["observation.images.agent"]["info"]["video.codec"] == "h264"
    assert result["features"]["observation.images.agent"]["shape"] == [480, 640, 3]
