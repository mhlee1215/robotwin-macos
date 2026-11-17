# Third-party notices

This is an independent macOS compatibility port of [RoboTwin](https://github.com/RoboTwin-Platform/RoboTwin), pinned to commit `ea8b21121ebb3cd201ff5b3fe361944ac94eda3f`.

The patch contains RoboTwin-derived code. The upstream MIT copyright and permission notice are preserved in [LICENSE](LICENSE), alongside the macOS additions.

RoboTwin source and official simulation assets are downloaded by the bootstrap script and are not redistributed in this repository. Asset licensing is governed by the [official dataset](https://huggingface.co/datasets/TianxingChen/RoboTwin2.0). Dependencies such as SAPIEN, PhysX, MoltenVK, PyTorch, Pinocchio, and TOPPRA retain their respective licenses. The MIT license here does not relicense those dependencies or assets.

Selected verification images were rendered from the upstream simulation assets on the tested Mac. JSON evidence retains measurements and source hashes; local filesystem path prefixes are replaced with `<PORT_ROOT>`, `<WORKSPACE>`, or `<HOME>`. Full videos, datasets, and intermediate diagnostics are not included.
