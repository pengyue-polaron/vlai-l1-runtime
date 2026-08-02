"""L1 composition of the shared collection interaction contract."""

from embodied_ops import CollectionInteraction
from embodied_ops.interaction import InputAction

L1_COLLECTION_INTERACTION = CollectionInteraction(
    input_actions=(
        InputAction("start", "Start recording", "\n", "primary"),
        InputAction("save", "Save episode", "\n", "primary"),
        InputAction("reset", "Reset position", "r\n", "quiet"),
        InputAction("discard", "Discard", "d\n", "danger"),
        InputAction("quit", "Quit", "q\n", "quiet"),
    ),
    start_action_ids=("start", "reset", "quit"),
    recording_action_ids=("save", "discard", "quit"),
)

__all__ = ["L1_COLLECTION_INTERACTION"]
