# Independent audit fixtures

These files are immutable copies used only by the test suite. Runtime code never
loads them; production inference loads only `models/model630_actor.onnx` and its
sidecar.

The public source export intentionally omits the `.pt`, `.onnx`, and replay
JSONL artifacts. Restore them from the separately controlled deployment assets
and verify the SHA256 values below before running artifact/parity or replay
tests.

- `model_630.pt`: source checkpoint, SHA256 `81501373c08f6f4aa6bae3fdaccdd073939e5346d2b25a8e419ab35c6657ed0c`
- `handoff_schema.json`: source handoff schema, SHA256 `f2bf9c51e37f675cdb6e063cab325c0bf0eb569bb46c0fb104c075c3e6ed7cee`
- `joint_normalization.json`: frozen V3 joint-normalization contract, SHA256 `31e60a3b3da7774cbb3aa749c54875824276b4f094485b8684029218f6d97170`
- `release_199.jsonl`: actor-only replay input, SHA256 `97e68ba897d48876bad304c997973368d37fb1a9b6240f9a0e22bee1d3dbfcac`
