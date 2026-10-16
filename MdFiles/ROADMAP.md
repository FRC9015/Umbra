# Umbra audit follow-up

Stage 1 established a local mathematical baseline and opt-in diagnostics.
The approved Stage 2 changes corrected B1/B2/B3 pose math and local invalid
observation handling. A status of
**Under investigation** means a permanent test
currently reproduces a concern, not that a production fix has begun.

| Audit ID | Finding | Status | Evidence or next check |
| --- | --- | --- | --- |
| B1 | Optical/NWU basis direction | Validated locally | `basis.*`, `fixed.*`, `config.*` pass; hardware check needed |
| B2 | Distortion applied twice | Validated | Observed corners now go directly to PnP with distortion; `optical.radial`, `.tangential`, `.edge` pass locally; hardware remains unverified |
| B3 | Extra 90° yaw | Validated locally | `yaw.*` pass; robot subscriber convention still needs verification |
| B4 | Tag corner order | Validated | `math.corner_order` and rotated-tag case pass synthetically; hardware check still needed |
| B5 | Camera calibration/mounts | Requires hardware testing | Config cases use current values; physical poses unverified |
| B6 | Field layout validation | Under investigation | Reference fields checked; startup failure handling untested |
| B7 | Pose quality/outliers/ambiguity | Not started | Clean planar case exists; adverse replay cases needed |
| B8 | Published distance semantics | Under investigation | `range.tag_distance` fails |
| B9 | Observation format/status | Partially implemented | Nine-element format retained; invalid/empty poses no longer published; subscriber must reject old values |
| C1 | NetworkTables publication period | Not started | Measure robot arrival rate |
| C2 | Capture/clock timestamps | Under investigation | Retrieval and call intervals instrumented; exposure/arrival unavailable |
| C3–C4 | Capture buffering and exposure | Requires hardware testing | Measure on Pi with actual camera modes |
| C5 | Preview overhead | Under investigation | Worker preview timing added; viewer-side profiling needed |
| C6 | Detector/OpenCV threading | Requires hardware testing | Benchmark on Pi under sustained load |
| C7 | Corner refinement/blur | Under investigation | Stage timings added; accuracy replay needed |
| C8 | Repeated work | Not started | Profile before optimizing |
| D1 | Tuning data race | Not started | Threaded route tests need later reliability stage |
| D2 | Configuration persistence | Not started | Fixture and concurrency tests need later reliability stage |
| D3 | Stale observations | Partially implemented | Local post-retrieval age gate and failure suppression; robot-consumer freshness check still needed |
| D4–D5 | Stream state and busy loop | Not started | Integration tests needed |
| D6–D7 | Recovery and errors | Not started | Failure injection/hardware checks needed |
| D8–D10 | Input, config, and web behavior | Not started | Route tests and schema checks needed |
| D11 | Image stride assumption | Not started | Padded-image integration test needed |
| D12 | Resource lifetime/shutdown | Not started | Controlled shutdown test needed |

Planned sequence: Stage 2 pose correctness; Stage 3 reliability; Stage 4
latency; Stage 5 pose quality; Stage 6 architecture; Stage 7 integrated
verification. Change statuses to **Implemented**, **Validated**, **Requires
hardware testing**, or **Deferred** only when supported by test results.
