# Point-cloud decode benchmark

Date: 2026-10-21  
Script: `benchmarks/pointcloud_benchmark.py`  
Raw result: `docs/benchmarks/pointcloud-decode-20261021.json`

## Environment and method

- Linux 5.15.136-tegra aarch64 (Tegra container), Python 3.10.12, NumPy 1.21.5.
- Isolated synthetic XYZ `sensor_msgs/PointCloud2` with 200,000 points, about 1%
  NaN/Inf, little-endian `FLOAT32`, `point_step=12`, `height=1`.
- One warmup pass per decoder, then 5 measured passes. `max_3d_points=60,000`.
- Old algorithm: per-point `struct.unpack` plus `math.isfinite` plus the exact
  historical `downsample_points` logic. This is equivalent to the previous
  `sensor_msgs_py.read_points` loop; `sensor_msgs_py` is not installed in this
  harness, so the benchmark uses the explicit equivalent Python loop.
- New algorithm: `web_control.pointcloud_runtime.parse_pointcloud2` (NumPy
  strided fields, vectorised finite mask, sampled indices before Python list
  materialisation).
- Both decoders produced **identical output** in the run: finite count
  196,912, 49,229 retained points, same last point.

This measures only isolated synthetic decoder CPU/method cost. **It is not
real-robot/node CPU, ROS/DDS cost, GIL contention, allocator behaviour under
live ROS traffic, or a sustained workload measurement.** The background worker
still shares the Python GIL, CPU and allocator with the node, and serializing
an HTTP response is separate JSON work.

## Results (per iteration)

| Metric | Old Python loop | New NumPy path | Ratio |
|---|---:|---:|---:|
| Wall time | 0.2372 s | 0.0516 s | 4.59x faster |
| Process CPU | 0.2362 s | 0.0515 s | 4.58x less |
| `tracemalloc` peak | 33.13 MB | 11.82 MB | 2.8x lower |

The end-to-end node saving depends on cloud size, field layout, topic rate, and
whether a map tab has requested the layer. When no cloud endpoint has requested
`local_map`, `obstacles`, or either Mapping map/ground cloud for
`cloud_demand_ttl_sec` (default 2 s), those decodes stop while at most the
latest raw frame is retained. A later `/api/map3d` or `/api/mapping/map3d`
request wakes the worker and decodes that latest frame. Only the three latched
navigation maps (`global_map`, `mapcloud`, `mapground`) are decoded once per new
input without a request for an early/late-browser cache. All layers retain only
the bounded 3D cache, not the raw ROS buffer.

The ROS executor no longer performs the synchronous point-cloud decode. Measure
the live node and machine after deployment with the existing resource logs;
this benchmark alone does not promise zero impact on control or navigation.

## Reproduce

```bash
python3 benchmarks/pointcloud_benchmark.py \
  --points 200000 --iterations 5 \
  --output docs/benchmarks/pointcloud-decode-20261021.json
```
