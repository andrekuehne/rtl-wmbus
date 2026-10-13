# Optimization strategy

This fork explores LLM-assisted performance improvements to rtl-wmbus. Aim for
measurable gains in small, independently reviewable patches suitable for upstream.
Treat generated code and performance claims as hypotheses requiring verification.
The primary goal is lower runtime resource use, especially CPU time and memory,
without reduced decoding quality.

## Issue-driven work

- Track each distinct optimization target as its own issue in
  `andrekuehne/rtl-wmbus`. Search existing issues before creating a duplicate.
- Build the backlog from code scans, profiling, and relevant GitHub discussions.
  Link original discussions and evidence; verify that suggested optimizations
  apply to this code rather than assuming their benefit.
- Each issue should identify the resource bottleneck or unmeasured hypothesis,
  target files/functions, proposed approach, affected behavior, dependencies,
  and a reproducible validation plan with measurable acceptance criteria.
- Process issues sequentially. Claim one issue before editing; its assigned agent
  owns investigation, implementation, validation, and integration into `dev`.
  Start the next issue only after integration or explicit deferral with a reason.
- Before editing, trace the relevant call paths and inspect current `dev`, recent
  changes, and other issue scopes. Choose the smallest coherent set of files and
  functions; record that scope in the issue and explain any necessary expansion.
- Prefer separate functions or pipeline stages to minimize overlap. When targets
  share code or state, record the dependency and sequence them accordingly.
  Avoid broad refactors or duplicated logic merely to prevent merge conflicts.
- Reference `andrekuehne/rtl-wmbus#<issue>` in commit bodies and upstream PRs to
  avoid cross-repository ambiguity. After integration, record validation,
  measurements, commit IDs, and upstream PR status in the issue, then close it
  when its acceptance criteria are met.

## Scope each change

- Work on one optimization hypothesis at a time. Identify the hotspot, expected
  benefit, affected behavior, and validation before editing.
- Prefer the smallest effective change. Follow nearby C style and existing C99
  conventions; avoid unrelated cleanup, renaming, formatting, or new dependencies.
- Separate prerequisite refactors, correctness fixes, and optimizations when they
  can stand alone. Do not build a large speculative rewrite or benchmark framework.
- Preserve CLI defaults, output format, streaming behavior, decoder state, and
  T1/C1/S1 reception. Keep portable builds usable on resource-constrained ARM devices.
- Floating-point reassociation, approximations, SIMD, and compiler flags can affect
  sensitivity or portability. Require evidence for those tradeoffs; retain a
  portable path and do not enable host-specific flags or fast math by default.

## Measure and validate

- Profile a release build on representative input before choosing an optimization.
  Save the parent revision's binary and output outside tracked files for comparison.
- Build with `make release` and `make debug`; compare warnings with the baseline.
  Both targets overwrite `build/rtl_wmbus`, so preserve the release binary before
  building debug and use release binaries for performance measurements.
- Replay the checked-in captures with identical options against baseline and
  candidate. Start with these documented configurations:

  ```sh
  build/rtl_wmbus < samples/rtlsdr_868.950M_1M6_samples2.cu8
  build/rtl_wmbus -o < samples/rtlsdr_868.950M_1M6_issue47.cu8
  build/rtl_wmbus -o < samples/rtlsdr_868.950M_1M6_issue49.cu8
  build/rtl_wmbus -d 3 -s -o < samples/rtlsdr_868.625M_2M4_issue48.cu8
  ```

- Compare packet payloads, order, duplicates, CRC/decoding status, RSSI, and exit
  status. Normalize only known nondeterministic metadata such as wall-clock
  timestamps and build identifiers; investigate other differences. Packet counts
  alone are insufficient. Exercise affected options and report missing coverage;
  matching these captures does not establish unchanged reception sensitivity.
- Benchmark baseline and candidate on the same machine with the same compiler,
  flags, input, and options. Warm up, repeat runs, and report typical CPU time or
  throughput, variation, and absolute and relative changes. Keep output handling
  identical. Do not claim gains within measurement noise or from debug builds.
- For memory-focused changes, measure peak memory use and allocations as relevant,
  and report CPU/memory tradeoffs rather than optimizing one metric in isolation.
- Add focused regression coverage when behavior or a subtle invariant needs it.
  Use sanitizers when memory, bounds, or arithmetic changes warrant them. Keep
  temporary captures, profiles, and build artifacts out of commits.
- If a change regresses correctness or has no credible benefit, revise or discard
  it. State limitations honestly; never present unrun checks as passing.

## Branches and upstream PRs

- Keep `master` aligned with upstream. Use `dev` as the integration branch for
  this fork's agent policy and accumulated, validated optimizations.
- Start each issue on a fresh `codex/<issue>-<topic>` branch from current `dev`.
  Profile, change, validate, and commit it, then integrate the validated commits
  into `dev` before starting the next issue. This becomes the next baseline.
- Prepare one upstream PR per logical optimization on a separate
  `codex/pr-<topic>` branch based on upstream's current default branch. Cherry-pick
  only the relevant code, tests, and evidence; exclude fork-specific agent policy
  and unrelated optimizations. Repeat correctness checks and benchmarks against
  that branch's upstream baseline before submission.
- Independent optimizations can be submitted as soon as they are ready. If a
  change depends on an earlier optimization, continue development locally and
  submit its PR after the prerequisite merges upstream.
- Keep PRs individually reviewable. Do not submit the accumulated `dev` branch
  as one large upstream PR.

## Keep a clean patch stack

- Inspect the working tree first and preserve unrelated user changes. Never
  rewrite existing commits without authorization.
- Commit each validated logical change before starting the next. Every commit
  should build and pass its relevant checks; avoid mixed-purpose or fixup-only
  commits in the final stack. Stack only necessary dependencies.
- Stage explicit paths and review the staged diff with `git diff --cached` and
  `git diff --cached --check`. Commit only the intended source, tests, and evidence.
- Use a short imperative subject. In the body, explain the problem, approach,
  compatibility implications, validation commands, and measured results when
  applicable. Make each patch understandable without this conversation.
- Keep this fork's agent policy in its own commit, separate from upstream code
  patches. Ensure patches can be cherry-picked individually or in documented order.
- For each completed change, report what changed, checks run, measured benefit,
  remaining limitations, and commit ID. Keep reports and proposed PRs concise.
