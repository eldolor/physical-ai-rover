# tools/trt: OWLv2's image tower as a TensorRT engine

**An experiment, not a shipped path.** The question was what INT8 costs in
accuracy against fp16 for OWLv2 on a Jetson (`PLAN-onboard-perception.md`
P7b assumed INT8 latency); only a TensorRT measurement predicts a Jetson.
Per the 3.76 P3 review, the file's known low-severity issues were not taken
because 3.41 is closed (`docs/plans/ros-alignment/3.76-legacy-review.md`).

| File | Purpose |
|---|---|
| `trt_owlv2.py` | `build_engine(onnx_path, out_path, precision="fp16"\|"int8", calib_path=None, workspace_gb=8)`: ONNX to a serialised engine (INT8 needs a calibration `.npy`, an entropy calibrator, and writes a `.calib` cache beside the engine). `TrtOwlv2(engine, model=..., threshold=0.0, device=None)`: the detector, image tower on TensorRT, text tower in PyTorch, behind the `Detector` protocol |

There is no command line. `TrtOwlv2` is reached through
`brain/perceive_lab.py`: `--detector trtowlv2:/path/to/owlv2_int8.engine`
(it raises without an engine path). It imports `tensorrt`, `torch` and
`transformers` lazily, so the suite never needs them, and it joins the
per-patch tensors to the text embedding with
`tools/hailo/owlv2_host_head.py` (the closed Hailo path's module, still
imported here).

No environment variables.

See `docs/engineering/perception/ENGINEERING.md` (the `brain/perceive_lab.py`
row names this loader).
