# Continuous integration

`workflows/ci.yml` runs on pull requests targeting `main` or `develop` and on
pushes to either branch. Its job is named `CI`; keep this name stable because
the branch protection rule for `main` requires that status check.

The job uses Python 3.12 and installs project dependencies with
`uv sync --extra test --extra lint --frozen`. Quality checks use `uv run
--no-sync` to preserve that environment. The build frontend is installed
separately in the runner's Python environment. The job builds a wheel and an
sdist, audits both, and checks that `uv.lock` has not changed.

## Branch protection

The GitHub protection rule for `main` requires:

- A pull request before merging, with no mandatory review approvals.
- A successful `CI` status check.
- The branch to be up to date before merging.
- Enforcement for administrators as well as other contributors.

Branch protection is configured in GitHub, not by the workflow file. After
uploading the workflow, open a pull request and confirm that `CI` appears and
that merging is blocked until it succeeds. For a new repository or fork,
configure the same rule under Settings > Branches.

If the workflow must be rolled back, first remove `CI` from the required checks
in that rule, then revert the workflow change. Otherwise pull requests will
remain blocked waiting for a check that can no longer run.

`workflows/publish.yml` is separate and runs only when a release is published.
