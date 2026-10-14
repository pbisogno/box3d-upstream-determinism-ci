# Box3D upstream test CI

This repository runs the existing public Box3D test suites from an unchanged, pinned upstream revision:

- Source: [erincatto/box3d](https://github.com/erincatto/box3d)
- Revision: `5d83df83ab47172c2c745b44315e7538ead02397`
- License: [upstream MIT license](https://github.com/erincatto/box3d/blob/5d83df83ab47172c2c745b44315e7538ead02397/LICENSE)

The workflow uses standard public GitHub-hosted runners on Windows x64, Linux x64, Linux ARM64 and macOS ARM64, with Debug and RelWithDebInfo builds. It runs the upstream `DeterminismTest` and `RecordingTest` in fresh processes, followed by the complete upstream test suite.

Checks use the existing upstream assertions and reference values. They do not establish exhaustive determinism or portable snapshots across every build. SIMD width is fixed to 4; upstream width comparisons may report a skip.

Only preinstalled tools are used. The workflow does not upload artifacts, create persistent caches, use paid larger runners or install packages. Public logs contain upstream build/test status and diagnostics.

See [GitHub Actions billing](https://docs.github.com/en/billing/concepts/product-billing/github-actions) and [standard hosted runners](https://docs.github.com/en/actions/reference/runners/github-hosted-runners).
