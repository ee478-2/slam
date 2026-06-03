# slam workspace (`catkin_ws/src/slam`)

This is the perception + SLAM + navigation workspace. It was split out of the
former monolithic `llm-skill` package during the 2026-06-03 restructure. The
agent stack now lives in the sibling `llm_agent/` package and the arm stack in
`manipulation_control/` — both are **outside this workspace's edit scope**.

## Workspace Scope Policy (hard constraint)

- **Edits, creation, and deletion are confined to `catkin_ws/src/slam/`.**
- Everything outside `slam/` — `llm_agent/`, `manipulation_control/`,
  `realsense-ros/`, and the rest of `catkin_ws/` — is **reference-only**:
  - **Read / reference:** allowed freely.
  - **Execute** (launch nodes, run scripts, query topics): allowed, but only
    **under explicit user permission** for each run.
  - **Modify** (edit/create/delete files): **forbidden.** If a cross-boundary
    change seems necessary, report it and get approval first — do not edit.

## Git Remote Policy (hard constraint)

- The user manages all git remotes manually. **The agent must never run
  remote-related git commands** — no `git push`, `git pull`, `git fetch`,
  `git clone`, `git remote add/set-url`, or any network git operation.
- Local-only git is allowed (`init`, `add`, `commit`, `status`, `diff`, `log`,
  `branch`, `checkout`). Commit locally; the user pushes when ready.

---

## Git Commit Policy

Commit code changes automatically as work progresses. Follow these rules strictly.

### When to commit
- After completing each logical unit of work (a feature, fix, or refactor — not every keystroke).
- After tests pass for the changed area.
- Before switching context to an unrelated task.
- Never end a session with a dirty working tree.

### Commit message format
Use Conventional Commits:
- `feat: <what was added>`
- `fix: <what was fixed>`
- `refactor: <what was restructured>`
- `perf: <performance change>`
- `docs: <doc changes>`
- `test: <test changes>`
- `chore: <tooling, deps, config>`

Rules:
- Subject ≤ 72 chars, imperative mood, no trailing period.
- For non-trivial changes, add a blank line and a body explaining **why**, not what.
- Scope prefix when useful: `feat(ppo): add observation masking`.

### Staging
- Prefer explicit paths or `git add -p`. Do **not** use `git add .` or `git add -A` without first reviewing `git status`.
- Keep commits atomic — one logical change per commit. Split unrelated edits.

### Pre-commit checklist
1. `git status` + `git diff --staged` to verify scope.
2. Run the project's test command if one exists.
3. Run linter / formatter if configured.
4. Abort the commit if any of the above fail; fix first, commit after.

### Never commit
- Secrets, API keys, tokens, `.env*` files.
- Large binaries, datasets, checkpoints, rosbags, model weights (>10 MB). Add to `.gitignore` instead.
- Debug prints, `TODO: remove`, or commented-out code left in by accident.
- Broken code to `main`. Use a WIP branch if you must checkpoint mid-work.

### Never do without explicit confirmation
- `git push --force` or `--force-with-lease` on shared branches.
- `git reset --hard`, `git clean -fdx`, or anything that discards uncommitted work.
- History rewrites (`rebase -i`, `commit --amend`) on already-pushed commits.
- Deleting or renaming branches.

### Attribution
Do not add `Co-authored-by: Claude` or any AI-attribution trailer to commit messages.

### Push policy
Do not push automatically. Commit locally; I'll push when ready.

---

## Progress Tracking

Maintain a persistent progress log at `docs/PROGRESS.md`. This is the project's single source of truth for "where are we?" — separate from git history (which is for *what changed*) and from CLAUDE.md (which is for *stable rules*).

### Session start
At the beginning of every session:
1. Read `docs/PROGRESS.md` — focus on the latest date entry and the `## Open` section.
2. If it doesn't exist yet, create it using the template below.
3. Briefly summarize the current state back to me before proposing next steps.

### When to update `PROGRESS.md`
- **Immediately after every git commit.** One commit → one progress bullet, referencing the short SHA.
- When an experiment finishes (whether it succeeded, failed, or was inconclusive — log all three).
- When a design decision is made. Record *what* and *why*, not just *what*.
- When you discover something non-obvious (a bug cause, a library quirk, a baseline number).
- When a blocker appears or is resolved.

### Update format
Append to the **top** of the file (newest first). Use this structure:

```markdown