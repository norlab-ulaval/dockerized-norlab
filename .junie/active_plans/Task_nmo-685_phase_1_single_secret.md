# Task NMO-685 — Phase 1 Simplification Plan: Use a Single Secret for All `chpasswd` Operations

References:
- Issue: Simplify Phase 1 to use only one Docker `secret` for all three password settings (project user, root, debugger user)
- Existing plan to adjust: `.junie/active_plans/Task_nmo-685_implementation_phase_1.md`
- Companion plan: `.junie/active_plans/Propose_me_a_plan_to_implement_nmo-685.md`

## Objective
Replace the "three separate secrets" approach with a single Docker secret (e.g., `dna_ssh_password`) used for:
- `echo "${DN_PROJECT_USER}:${SSH_PASSWORD}" | chpasswd`
- `echo "root:${SSH_PASSWORD}" | chpasswd`
- `echo "${DN_SSH_SERVER_USER}:${SSH_PASSWORD}" | chpasswd` (when debugger user is enabled)

Maintain a temporary environment variable fallback for backward compatibility during migration, but prioritize the secret.

## Summary of Changes
- In documentation and code snippets, replace `ssh_user_password`, `ssh_project_user_password`, and `ssh_root_password` with a single secret: `dna_ssh_password`.
- Adjust the `dn_install_debugging_tools.bash` snippet to read one secret and apply it to project user, root, and debugger user.
- Update `docker-compose.dn-project.build.yaml` examples to define only one `build.secrets` entry.
- Update the `Dockerfile` example to mount only `dna_ssh_password` via BuildKit and export it to satisfy existing preconditions.
- Simplify the `secrets/` directory setup to a single file.

## Detailed Implementation Steps

### 1) Update `dn_install_debugging_tools.bash` snippet (documentation change)
Replace the current Phase 1 snippet in `.junie/active_plans/Task_nmo-685_implementation_phase_1.md` with the following single‑secret version:

```bash
# ....Set password for users.......................................................................
# Prefer Docker secret, fallback to env for backward compatibility
if [[ -f /run/secrets/dna_ssh_password ]]; then
  SSH_PASSWORD=$(cat /run/secrets/dna_ssh_password)
else
  SSH_PASSWORD="${DN_SSH_SERVER_USER_PASSWORD:-}"
fi

# Validate that a password is available
if [[ -z "${SSH_PASSWORD}" ]]; then
  echo "ERROR: SSH password not available from secret or environment" >&2
  exit 1
fi

# Set passwords using the single secret
echo "${DN_PROJECT_USER}:${SSH_PASSWORD}" | chpasswd
echo "root:${SSH_PASSWORD}" | chpasswd

# ....Create and setup specialized debugger user.................................................
if [[ ${_SETUP_DEBUGGER_USER} == true ]]; then
  useradd --gid "${DN_PROJECT_GID:?err}" --create-home "${DN_SSH_SERVER_USER}" || exit 1
  echo "${DN_SSH_SERVER_USER}:${SSH_PASSWORD}" | chpasswd
  # (keep the rest of the debugger user setup steps as-is)
fi
```

Notes:
- This preserves backward compatibility by keeping `DN_SSH_SERVER_USER_PASSWORD` as a fallback while the ecosystem migrates to secrets.
- The precondition in `dn_install_debugging_tools.bash` currently checks `DN_SSH_SERVER_USER_PASSWORD` is non-empty. To avoid changing that check during Phase 1, we will export the variable from the secret during the Dockerfile build step (see step 3). This keeps changes minimal.

### 2) Update Docker Compose for build (single secret)
File: `dockerized-norlab-images/core-images/dn-project/docker-compose.dn-project.build.yaml`

Replace the three secret IDs and sources with a single `dna_ssh_password` secret under the `project-develop-main.build.secrets` section:

```yaml
services:
  project-develop-main:
    build:
      context: project-develop
      dockerfile: Dockerfile
      secrets:
        - id: dna_ssh_password
          src: ./secrets/dna_ssh_password.txt
      args:
        # Do not pass DN_SSH_SERVER_USER_PASSWORD via build args
        BASE_IMAGE: ${DN_PROJECT_HUB:?err}/${DN_PROJECT_IMAGE_NAME:?err}-core
        BASE_IMAGE_TAG: ${PROJECT_TAG:?err}
# Build-time secrets definition is inlined above via id/src. No service-level secrets needed for Phase 1.
```

### 3) Update the project-develop Dockerfile usage (BuildKit secret mount)
File: `dockerized-norlab-images/core-images/dn-project/project-develop/Dockerfile`

- Remove the default password value and keep the ARG (optional) without default for legacy builds:

```dockerfile
# Remove this line completely:
# ARG DN_SSH_SERVER_USER_PASSWORD=*****

# Optionally keep (no default):
ARG DN_SSH_SERVER_USER_PASSWORD
```

- Mount the single secret and export it to satisfy the precondition check before sourcing the setup script:

```dockerfile
RUN --mount=type=secret,id=dna_ssh_password,required=1 \
    export DN_SSH_SERVER_USER_PASSWORD="$(cat /run/secrets/dna_ssh_password)" && \
    source ./dn_install_debugging_tools.bash && \
    rm -f ./dn_install_debugging_tools.bash
```

Rationale:
- The script currently checks that `DN_SSH_SERVER_USER_PASSWORD` is set. Exporting it from the secret keeps the script unchanged while enabling secret-based provisioning.

### 4) Secrets directory setup (single file)

```bash
# Create secrets directory (add to .gitignore)
mkdir -p secrets/

# Generate one strong password for Phase 1
openssl rand -base64 32 > secrets/dna_ssh_password.txt

# Secure the secret
chmod 600 secrets/dna_ssh_password.txt
```

### 5) .gitignore update (unchanged from prior plan)

```gitignore
# Docker secrets
secrets/
*.secret
```

### 6) Verification steps
- Build images:
  - `dna build project-develop` (or the equivalent build invocation used in this repo)
  - Confirm the build succeeds without passing any password as a build arg.
- Run container (dev config):
  - Ensure port `2222` is mapped as per `global/docker-compose.global.yaml`.
- Validate SSH access and passwords inside the container:
  - Attach to the container and run: `getent shadow | grep -E "^(root|${DN_PROJECT_USER}|${DN_SSH_SERVER_USER}):"` to verify entries exist (do not print secrets).
  - Attempt SSH login as `${DN_SSH_SERVER_USER}` on port `2222` using the generated password; confirm access.
- Ensure no password is printed in any logs (entrypoint already avoids this).

### 7) Backward compatibility and deprecation
- During Phase 1, if `dna_ssh_password` secret is absent, operators may still pass `DN_SSH_SERVER_USER_PASSWORD` via build arg to unblock urgent builds. However, the recommended path is to use the secret.
- Plan to remove the env fallback once all environments provide the secret (Phase 1d/Phase 2).

## Acceptance Criteria
- Only a single secret (`dna_ssh_password`) is used throughout Phase 1 documentation and examples.
- Builds succeed with BuildKit secret mount and without leaking passwords into image history.
- The script sets the same password for project user, root, and debugger user using the one secret.
- No credentials are logged to stdout/stderr.

## Out of Scope (Phase 2+)
- Per-user unique passwords via multiple secrets (revisit later if needed).
- Defaulting to SSH key-based authentication and disabling password auth by default (tracked for Phase 2).

## Rollback Strategy
- If issues arise, temporarily restore the environment variable pathway by passing `--build-arg DN_SSH_SERVER_USER_PASSWORD=...` while the single secret pipeline is fixed.
