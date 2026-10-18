# Capoworks checks

`capoworks-checks.yaml` runs on pushes to `master`, pull requests targeting
`master` or `port/**`, and manual dispatch. The `port/**` target keeps stacked
PRs covered. All jobs use GitHub-hosted Ubuntu 24.04 runners.

| Check | Coverage |
| --- | --- |
| `Capoworks / unit-tests` | Clean `fome_test` build with sanitizers, followed by the test suite and a zero-failure GoogleTest XML check |
| `Capoworks / simulator` | Linux simulator build and a 10-second smoke run, with a 60-second timeout |
| `Capoworks / core8` | Core8 firmware and OpenBLT build, illegal time-conversion check, required output files |

The final `Capoworks checks` job succeeds only when every matrix job succeeds.
Select that check in the branch protection rules if it should be required for
merging. The workflow does not change repository settings.

Each job uploads its log, exit status and any GoogleTest XML reports, including
on failure. A successful Core8 build also uploads the combined image, application
update, bootloader and matching TunerStudio INI. Artifacts are retained for 14 days.

## Run locally

On Linux, initialize the repository submodules and install Java 21, a C/C++ build
toolchain, `gcc-multilib`, `g++-multilib`, `mtools`, `dosfstools`, `zip`, `xxd` and `rsync`.
The ARM compiler and checksum tools come from `firmware/ext/build-tools`.
The harness gives the board script local `objcopy`/`objdump` aliases for those ARM
tools. Their directory must stay aligned with `firmware/use_arm_gcc.mk` when the
bundled toolchain changes.

```bash
git submodule update --init
bash misc/ci/capoworks-checks.sh all
# Or run one check:
JOBS=4 bash misc/ci/capoworks-checks.sh unit-tests
bash misc/ci/capoworks-checks.sh simulator
bash misc/ci/capoworks-checks.sh core8
```

Commands work from any directory. Host builds default to 12 jobs; GitHub uses 4.
The existing Core8 board script uses 20 jobs internally. Each check cleans its
build outputs and shared precompiled headers before building. Core8 also replaces
`firmware/deliver/`, as its board script normally does. Do not run builds concurrently
in the same worktree. Generated configuration files can change during these builds.

Logs and reports go to the ignored `build/capoworks-checks/<target>/` directory.
`all` runs sequentially and stops at the first failure. These checks build and run
host tests; hardware validation requires the separate hardware CI setup.
