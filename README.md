# Box3D upstream test CI

This repository builds an unchanged public Box3D revision and exercises its existing tests and public recording APIs.

- Source: [erincatto/box3d](https://github.com/erincatto/box3d)
- Revision: `5d83df83ab47172c2c745b44315e7538ead02397`
- License: MIT; see [LICENSE](LICENSE) and the [upstream license](https://github.com/erincatto/box3d/blob/5d83df83ab47172c2c745b44315e7538ead02397/LICENSE). The upstream notice is retained for the adapted example; the generic CI glue is provided under the same MIT terms by its contributors.

The workflow uses standard public GitHub-hosted Windows x64, Linux x64/ARM64 and macOS x64/ARM64 runners with preinstalled tools. It records CPU manufacturer/model and instruction capabilities visible to the runner, without host identifiers.

Debug and RelWithDebInfo builds run the unchanged `DeterminismTest`, `RecordingTest` and complete upstream suite in fresh processes. SIMD width 4 is the common profile. AVX2 variants on x64 require observable AVX2 support and actual execution of the upstream SIMD-width and cross-width replay comparisons; a skipped comparison fails those variants.

`portable-replay.c` is a generic public-API example adapted from the upstream `MidStreamContacts` scene. It records 128 updates after a contact checkpoint, verifies replay and seek, and reports rejection or first divergence when available through the API. Four producer jobs exchange only these public recordings; each consumer replays all four received files and tests rejection of a bad-magic copy.

Recordings travel as size-bounded compressed job outputs. The receiver verifies the original uncompressed byte count and SHA-256 before replay; the original recording bytes are preserved. No artifacts, persistent caches, packages, LFS or paid larger runners are used, and no tools are installed.

These checks exercise public upstream assertions and selected state hashes. They do not establish exhaustive determinism or equality of every causal-state byte. Replay rejection alone does not identify the exact incompatibility; CPU properties are those exposed to the runner and do not guarantee a specific future host.

Public logs contain generic build/test status, CPU capabilities and public-example results. Actions may render the compressed public recordings when showing the transfer environment; only the published example data is transferred. See [GitHub Actions billing](https://docs.github.com/en/billing/concepts/product-billing/github-actions), [standard hosted runners](https://docs.github.com/en/actions/reference/runners/github-hosted-runners) and [job-output limits](https://docs.github.com/en/actions/reference/workflows-and-actions/workflow-syntax#jobsjob_idoutputs).
