# Third-party notices

This project's own code is licensed under the AGPL-3.0-only (`LICENSE`). The
parts below are not; each keeps its own licence, and this project's
licence does not change their terms. Checked 2026-11-02 against each
project's published licence; re-check before a release.

## Included in this repository

| Part | Where | Licence |
|---|---|---|
| ProcTHOR-10K floor plans (Allen Institute for AI) | `sim/maps/procthor_data/` | Apache-2.0 (`sim/maps/procthor_data/LICENSE`) |
| Patch to Waveshare's UGV ESP32 firmware (`ugv_base_ros`) | `firmware/ugv_base_ros/` | GPL-3.0 only (a modification of GPL-3.0 firmware) |
| Simulated motor board reimplementing Waveshare's `ugv_base_ros` firmware behaviour | `sim/fake_esp32.py` | GPL-3.0-only (reimplements GPL-3.0 firmware) |
| LDROBOT's CRC-8 table (`ldlidar_stl_ros2`, `lipkg.cpp` `CrcTable`) | `robot/lidar_ld19.py` (`CRC_TABLE`) | MIT, Copyright (c) 2020-2021 SHENZHEN LDROBOT CO., LTD. (`LICENSES/MIT-LDROBOT.txt`) |
| Lines of NVIDIA TensorRT-Edge-LLM v0.9.1 source, quoted as patch anchors | `tools/jetson/patch_edgellm_trt103.py` | Apache-2.0, Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES (`LICENSES/Apache-2.0.txt`) |

## Downloaded or installed at run time (not included)

### Restrict commercial use

These matter for anyone selling or hosting a product built on this project,
whatever licence they hold for this project's code.

| Part | Used for | Licence | Commercial use |
|---|---|---|---|
| Ultralytics (YOLOE-11s-seg and its weights) | on-board detection (`brain/perceive.py`) | AGPL-3.0, or an Ultralytics Enterprise licence | Allowed under AGPL-3.0 with full source; otherwise buy Ultralytics' licence |
| Qwen2.5-VL-3B-Instruct (and its AWQ build, engine VE) | local identity check (`service/local_vlm/`) | Qwen Research License | **Non-commercial only**; commercial use needs a licence from Alibaba Cloud, or another model |
| Apple MobileCLIP weights (YOLOE's text encoder, `mobileclip_blt`) | encoding target text | Apple Machine Learning Research Model licence | **Research only**; confirm the terms Ultralytics redistributes them under |
| NVIDIA SegFormer-B0 (ADE20K) | optional floor mask (`brain.perception_floor_mask`, off by default) | NVIDIA Source Code License (SegFormer) | **Non-commercial only** |
| NVIDIA TensorRT, TensorRT Edge-LLM, JetPack | the local VLM's runtime on the Jetson | NVIDIA's licences (TensorRT SDK licence; Edge-LLM per its repository) | Redistribution under NVIDIA's terms |

### Permissive or weak copyleft

| Part | Licence |
|---|---|
| OpenCLIP, OpenAI CLIP RN50 weights | MIT |
| PyTorch, torchvision | BSD-3-Clause |
| Hugging Face Transformers | Apache-2.0 |
| ROS 2 Humble, nav2, `ros2_control`, `twist_mux` | Apache-2.0 |
| `slam_toolbox` | LGPL-2.1 |
| py_trees | BSD-3-Clause |
| FastAPI, Uvicorn, Pydantic, HTTPX | MIT / BSD-3-Clause |
| NumPy, SciPy, Pillow, PyYAML | BSD / MIT-CMU / MIT |
| Anthropic and AWS SDKs (`anthropic`, `boto3`) | MIT / Apache-2.0 |
| llama.cpp (benchmarks only) | MIT |

Cloud model calls (Anthropic, Amazon Bedrock) are governed by those
providers' service terms, not by a software licence.

## Not used

Waveshare's stock Jetson application (`ugv_jetson`, AGPL-3.0) and its ROS
nodes are disabled on the robot and no code from them is included.
