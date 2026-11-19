"""3.189: the ROS image's Dockerfile builds on the stock base unless told otherwise."""
import re
import subprocess
from pathlib import Path

SLAM = Path(__file__).resolve().parent.parent / "service" / "slam"


def test_default_base_is_the_stock_ros_image():
    text = (SLAM / "Dockerfile").read_text()
    assert re.search(r"^ARG BASE_IMAGE=ros:humble-ros-base$", text, re.M)
    from_lines = [l for l in text.splitlines() if l.startswith("FROM ")]
    assert from_lines == ["FROM ${BASE_IMAGE}"]


def test_cloud_build_script_refuses_latest_and_keeps_ca_out_of_the_repo():
    script = (SLAM / "build-cloud.sh").read_text()
    assert "mktemp -d" in script and "BASE_IMAGE=" in script
    assert not list(SLAM.rglob("*.crt")) and not list(SLAM.rglob("*.pem"))


def test_cloud_build_script_refuses_the_shared_tag_before_touching_docker():
    for tag in ("vision-picar-ros:latest", "vision-picar-ros"):
        r = subprocess.run(["bash", str(SLAM / "build-cloud.sh"), tag],
                           capture_output=True, text=True, env={"PATH": "/usr/bin:/bin"})
        assert r.returncode == 2, (tag, r.stdout, r.stderr)
