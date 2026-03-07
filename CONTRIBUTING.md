# Contributing to modbus_ros2_control

Thank you for your interest in contributing. Whether it's a bug report, feature, or documentation fix, contributions are welcome.

Please read this document before submitting issues or pull requests.

## Reporting bugs and feature requests

Use the [GitHub issue tracker](https://github.com/YOUR_ORG/modbus_ros2_control/issues) to report bugs or suggest features.

When filing an issue, please:

- Check existing [open](https://github.com/YOUR_ORG/modbus_ros2_control/issues) and [recently closed](https://github.com/YOUR_ORG/modbus_ros2_control/issues?q=is%3Aissue+is%3Aclosed) issues first.
- Include a reproducible test case or steps.
- Mention the version of the code and your environment (OS, ROS 2 distro).

## Code style and formatting

- **C++**: Use [clang-format](https://clang.llvm.org/docs/ClangFormat.html) with the repo’s `.clang-format` (ROS 2 / Google-based style).
- **YAML / Python**: Keep consistent indentation and line length.

### Format C++ locally

```bash
# Format all C/C++ files in the package
find . -type f \( -name '*.cpp' -o -name '*.hpp' -o -name '*.c' -o -name '*.h' \) ! -path './.git/*' -exec clang-format-14 -i --style=file {} +
```

Install clang-format (Ubuntu):

```bash
sudo apt install clang-format-14
```

## Pre-commit hooks (optional)

To run format and basic checks before each commit:

1. Install pre-commit: `pip install pre-commit`
2. Install hooks: `pre-commit install`
3. Run on all files: `pre-commit run --all-files`

You need `clang-format-14` (or adjust `.pre-commit-config.yaml` for another version).

## Pull requests

Before opening a PR:

1. Base your work on the latest `main` (or `jazzy`) branch.
2. Run clang-format on changed C++ files (or use pre-commit).
3. Ensure the package builds: `colcon build --packages-select modbus_ros2_control`
4. For larger changes, open an issue first to discuss.

To submit a PR:

1. Fork the repository.
2. Create a branch from `main` (or `jazzy`).
3. Make your changes and commit with clear messages.
4. Push and open a pull request.
5. Address any CI failures and review feedback.

CI runs build/test and a format check; both must pass.

## License

By contributing, you agree that your contributions will be licensed under the [Apache License 2.0](LICENSE).
