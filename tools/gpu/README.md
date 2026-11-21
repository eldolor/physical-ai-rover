# tools/gpu: rented-GPU corpus scoring (HISTORICAL)

> **`ec2.sh` and `eval_host.sh` start PAID EC2 hosts (a g5 GPU, about $1 an
> hour; an unattended one is about $29 a day).** Spending is the user's
> call: a session never runs them unasked (`CLAUDE.md` section 3c;
> `docs/guides/PARALLEL-SESSIONS.md` rule 10 keeps "spending" on the list
> of things asked first). Read `down` before `up`. A failed `up` names the
> instance it left running; the fix is `bash tools/gpu/ec2.sh down`.

**What this is for.** Scoring the labelled walk corpus
(`control/perception_eval.py`) on a rented A10G instead of a laptop, so a
sweep of a dozen configs takes an hour. It is **historical**: the P7 runs of
2026-09-11/12 (`PLAN-onboard-perception.md` P7). Nothing on a current path
uses it. Its numbers compare configs to each other; an A10G says nothing
about the Jetson's budget.

**Access.** No key pair and no inbound rule: SSM Session Manager over the
instance's own outbound HTTPS. The instance profile can read one S3 prefix,
not the account. Needs the `aws` CLI with a valid session.

## Scripts

| Script | Purpose | Run |
|---|---|---|
| `ec2.sh` | The host's lifecycle: launch, push corpus and code through S3, set up a venv, run the soft-gate sweep, pull records, tear down | `tools/gpu/ec2.sh <up\|status\|push\|setup\|sweep\|logs\|pull\|run "cmd"\|shell\|down>`; no argument prints its usage |
| `eval_host.sh` | The same host for `perception_eval`'s full-corpus matrix. It **delegates the host lifecycle to `tools/hailo/ec2.sh`** (the closed Hailo path's script; it maps the `GPU_*` variables to `HAILO_*`) and arms a 4 h dead-man switch (`shutdown -h +240`) at `up` | `tools/gpu/eval_host.sh <up\|setup\|push\|matrix\|pull\|status\|down>` |
| `run_matrix.sh` | The sweep `eval_host.sh matrix` runs on the box: every YOLOE variant, with and without the floor mask, plus OWLv2 as control, over the whole corpus | Not run by hand; `eval_host.sh matrix` calls it. It calls `tools/hailo/ec2.sh run` |
| `sweep.py` | The soft-gate sweep (`brain/perceive.py`'s `soft_gate`, k between the hard gate and open vocabulary): one model instance, every config | On the box: `python sweep.py --out out --device cuda [--ks 1,4,8] [--with-floor] [--only ...] [--fp-budgets 0,3,16] [--recordings recordings]`; `ec2.sh push` ships it and `ec2.sh sweep [args]` starts it under `setsid` |
| `pacing_sweep.py` | Phase D: the cold-search interval chosen by measuring look latency (frames from a target's first visible frame to the first cloud call), with the cloud stubbed. Distance rows use derived odometry | On the box: `python pacing_sweep.py --out out --device cuda [--frames 2,3,4,6,9,12] [--cms 20,40,80] [--cloud-frames 4] [--nominal-cm N] [--recordings recordings]` |

`ec2.sh` also has `logs [N]` (tail the sweep log; not in its usage text).
`ec2.sh down` leaves `s3://$GPU_BUCKET/$GPU_PREFIX/` in place.

## Variables

All are optional. `ec2.sh` defaults (read from the script):

| Variable | Default | Effect |
|---|---|---|
| `AWS_REGION` | `us-east-2` | The region of every call |
| `GPU_HOST_NAME` | `vision-picar-gpu-eval` | The instance's Name tag, and the name of its role, profile and security group |
| `GPU_INSTANCE_TYPE` | `g5.xlarge` | What `up` launches |
| `GPU_VOLUME_GB` | `150` | Root volume (gp3, encrypted, deleted on termination) |
| `GPU_BUCKET` | `vision-picar-deploy-303351622021-us-east-2` | The S3 bucket the corpus and code travel through |
| `GPU_PREFIX` | `gpu-eval` | The one prefix the instance profile may read and write |
| `GPU_OUT` | `evaluations/gpu/softgate` | Where `pull` puts the records |
| `GPU_HOURLY` | `1.006` | **Cost readout only**: change it with `GPU_INSTANCE_TYPE` or `status` prints a wrong figure |
| `GPU_SPOT` | `0` | `1`: a one-time spot instance, terminated on interruption; work must checkpoint to S3 and be relaunched with `up` (3.96) |
| `GPU_PURPOSE` | `perception-eval` | The instance's `Purpose` tag |
| `GPU_AZ` | unset (first default subnet) | Pick the availability zone (spot capacity differs by zone) |
| `GPU_AMI` | the latest Deep Learning base AMI, resolved through SSM | Pin an image |

`eval_host.sh` reads the same names with other defaults: `GPU_INSTANCE_TYPE`
`g5.2xlarge`, `GPU_VOLUME_GB` `120`, `GPU_HOURLY` `1.212`, `GPU_OUT`
`evaluations/gpu-yoloe`, `GPU_AMI` the latest Deep Learning OSS Nvidia
Driver PyTorch AMI; plus `CORPUS_BUCKET`
(`vision-picar-recordings-303351622021-us-east-2`, read-only, granted to the
role and removed again by `down`).

## Where the rest is written

`docs/engineering/perception/ENGINEERING.md` (the `tools/gpu/` row: what
3.76 P3 fixed); `PLAN-onboard-perception.md` P7 (read it before re-using
these). `docs/engineering/operations/ENGINEERING.md` covers the repo's
other AWS scripts.
