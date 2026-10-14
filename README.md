# Box3D upstream test CI

This repository builds an unchanged public Box3D revision and exercises its existing tests, public recording APIs and generic public example scenes.

- Source: [erincatto/box3d](https://github.com/erincatto/box3d)
- Revision: `5d83df83ab47172c2c745b44315e7538ead02397`
- License: MIT; see [LICENSE](LICENSE) and the [upstream license](https://github.com/erincatto/box3d/blob/5d83df83ab47172c2c745b44315e7538ead02397/LICENSE). The upstream notice is retained for the adapted example; the generic CI glue is provided under the same MIT terms by its contributors.

The workflows use standard public GitHub-hosted Windows x64, Linux x64/ARM64 and macOS x64/ARM64 runners with preinstalled tools. They record CPU manufacturer/model and instruction capabilities visible to the runner, without host identifiers.

The manually dispatched upstream-test workflow runs Debug and RelWithDebInfo builds of unchanged `DeterminismTest`, `RecordingTest` and the complete upstream suite in fresh processes. SIMD width 4 is the common profile. AVX2 variants on x64 require observable AVX2 support and actual execution of the upstream SIMD-width and cross-width replay comparisons; a skipped comparison fails those variants.

`portable-replay.c` is a generic public-API example adapted from the upstream `MidStreamContacts` scene. It records 128 updates after a contact checkpoint, verifies replay and seek, and reports rejection or first divergence when available through the API. Four producer jobs exchange only these public recordings; each consumer replays all four received files and tests rejection of a bad-magic copy.

The additional manually dispatched scene workflow has five producer origins, including macOS Intel. `stress-replay.c` uses public APIs for three small contacts, joints and lifecycle scenes adapted from published upstream examples. Each scene runs 4096 updates and records a 128-update window. Explicitly selected public values are encoded as tagged IEEE-754 bits or integer values, with no epsilon or quantization. The field list and omissions are declared in the C source; this is a projection of observations.

The scene workflow verifies original reference values during replay, repeated restart/backward-seek/continuation and fresh-player cycles. Consumer profiles request simulation workers 1/2/4, Debug/RelWithDebInfo and AVX2 OFF/ON where supported; each restore checks the actual worker count. Build parallelism is a separate option. Format rejection, physical divergence and failed execution gates have separate statuses. A full run checks all three cases before failing the job if any case failed.

`layout/` compiles a generic sizeof/offsetof inspector against published upstream internal headers without linking or changing the engine. Those structs describe the pinned recording format; they are not a supported external ABI.

Recordings and selected-value references travel as bounded compressed job outputs. Each receiving step takes one origin and one case, split into six lossless chunks of at most 20000 ASCII characters each; a `.` marks an empty tail chunk. The complete envelope is capped at 120000 characters. The receiver joins the chunks into a temporary runner file, and replay validates each original byte count and SHA-256. Recording and reference bytes are preserved. Receiving one origin at a time bounds the process environment on all platforms. No artifacts, persistent caches, packages, LFS or paid larger runners are used, and these workflows install no tools.

These checks exercise public upstream assertions, selected state hashes and an explicit projection of public values. They do not establish exhaustive determinism or equality of every causal-state byte. Whole-run projection digests and 512-state block digests supplement the transferred reference window. Replay rejection alone does not identify the exact incompatibility; CPU properties are those exposed to the runner and do not guarantee a specific future host.

Public logs contain generic build/test status, CPU capabilities and public-example results. The scene producers also log a bounded, lossless projection of the first 512 states in chunks to support field diagnostics; this does not change comparison gates. Actions may render the compressed public recordings when showing the transfer environment; only the published example data is transferred. See [GitHub Actions billing](https://docs.github.com/en/billing/concepts/product-billing/github-actions), [standard hosted runners](https://docs.github.com/en/actions/reference/runners/github-hosted-runners) and [job-output limits](https://docs.github.com/en/actions/reference/workflows-and-actions/workflow-syntax#jobsjob_idoutputs).
