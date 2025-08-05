## Updated Lines for Phase 1 Implementation
References:
- NMO-685 fix: implement secret best-practice
- `.junie/plans/Propose_me_a_plan_to_implement_nmo-685.md`

Here's how you should update those lines to implement Phase 1 of the Docker secret management best practices:

### Original Code (Lines 78-86):
```bash
# ....Set password for users.......................................................................
# user:newpassword
echo "${DN_PROJECT_USER}:${DN_SSH_SERVER_USER_PASSWORD}" | chpasswd
echo "root:${DN_SSH_SERVER_USER_PASSWORD}" | chpasswd

# ....Create and setup specialized debugger user.................................................
if [[ ${_SETUP_DEBUGGER_USER} == true ]]; then
  useradd -m "${DN_SSH_SERVER_USER}" \
    && yes "${DN_SSH_SERVER_USER_PASSWORD}" | passwd "${DN_SSH_SERVER_USER}"
```

### Updated Code for Phase 1:
```bash
# ....Set password for users.......................................................................
# Read passwords from Docker secrets instead of environment variables
if [[ -f /run/secrets/ssh_user_password ]]; then
  SSH_USER_PASSWORD=$(cat /run/secrets/ssh_user_password)
elif [[ -f /run/secrets/ssh_project_user_password ]]; then
  SSH_PROJECT_USER_PASSWORD=$(cat /run/secrets/ssh_project_user_password)
else
  # Fallback to environment variable for backward compatibility (temporary)
  SSH_USER_PASSWORD="${DN_SSH_SERVER_USER_PASSWORD:-}"
  SSH_PROJECT_USER_PASSWORD="${DN_SSH_SERVER_USER_PASSWORD:-}"
fi

if [[ -f /run/secrets/ssh_root_password ]]; then
  SSH_ROOT_PASSWORD=$(cat /run/secrets/ssh_root_password)
else
  # Fallback to environment variable for backward compatibility (temporary)
  SSH_ROOT_PASSWORD="${DN_SSH_SERVER_USER_PASSWORD:-}"
fi

# Validate that passwords are available
if [[ -z "${SSH_PROJECT_USER_PASSWORD}" ]] || [[ -z "${SSH_ROOT_PASSWORD}" ]]; then
  echo "ERROR: SSH passwords not available from secrets or environment variables" >&2
  exit 1
fi

# Set passwords using secrets
echo "${DN_PROJECT_USER}:${SSH_PROJECT_USER_PASSWORD}" | chpasswd
echo "root:${SSH_ROOT_PASSWORD}" | chpasswd

# ....Create and setup specialized debugger user.................................................
if [[ ${_SETUP_DEBUGGER_USER} == true ]]; then
  # Validate debugger user password
  if [[ -z "${SSH_USER_PASSWORD}" ]]; then
    echo "ERROR: SSH debugger user password not available from secrets or environment variables" >&2
    exit 1
  fi
  
  useradd -m "${DN_SSH_SERVER_USER}" \
    && echo "${DN_SSH_SERVER_USER}:${SSH_USER_PASSWORD}" | chpasswd
```

### Additional Changes Required:

#### 1. Update the Dockerfile (remove hardcoded password):
```dockerfile
# Remove this line completely:
# ARG DN_SSH_SERVER_USER_PASSWORD=lasagne

# Replace with:
ARG DN_SSH_SERVER_USER_PASSWORD
```

#### 2. Update docker-compose.dn-project.build.yaml:
```yaml
services:
  project-develop-main:
    secrets:
      - ssh_user_password
      - ssh_project_user_password  
      - ssh_root_password
    build:
      context: project-develop
      dockerfile: Dockerfile
      secrets:
        - ssh_user_password
        - ssh_project_user_password
        - ssh_root_password
      args:
        # Remove DN_SSH_SERVER_USER_PASSWORD from build args
        BASE_IMAGE: ${DN_PROJECT_HUB:?err}/${DN_PROJECT_IMAGE_NAME:?err}-core
        BASE_IMAGE_TAG: ${PROJECT_TAG:?err}

secrets:
  ssh_user_password:
    file: ./secrets/ssh_user_password.txt
  ssh_project_user_password:
    file: ./secrets/ssh_project_user_password.txt
  ssh_root_password:
    file: ./secrets/ssh_root_password.txt
```

#### 3. Create secrets directory structure:
```bash
# Create secrets directory (add to .gitignore)
mkdir -p secrets/

# Generate strong passwords
openssl rand -base64 32 > secrets/ssh_user_password.txt
openssl rand -base64 32 > secrets/ssh_project_user_password.txt  
openssl rand -base64 32 > secrets/ssh_root_password.txt

# Secure the secrets
chmod 600 secrets/*.txt
```

#### 4. Update .gitignore:
```gitignore
# Docker secrets
secrets/
*.secret
```

#### 5. Update the entrypoint script (dn_entrypoint.init.bash line 20):
```bash
# Replace this line:
n2st::print_msg "Starting container internal ssh server for IDE remote development workflow on port ${MSG_DIMMED_FORMAT}${DN_SSH_SERVER_PORT}${MSG_END_FORMAT} with user ${MSG_DIMMED_FORMAT}${DN_SSH_SERVER_USER}${MSG_END_FORMAT} (default pass: lasagne)"

# With this (remove password disclosure):
n2st::print_msg "Starting container internal ssh server for IDE remote development workflow on port ${MSG_DIMMED_FORMAT}${DN_SSH_SERVER_PORT}${MSG_END_FORMAT} with user ${MSG_DIMMED_FORMAT}${DN_SSH_SERVER_USER}${MSG_END_FORMAT}"
```

### Key Security Improvements:

1. **Eliminated Hardcoded Passwords**: No more "lasagne" password in the Dockerfile
2. **Docker Secrets Integration**: Passwords are now read from Docker secrets
3. **Separate Passwords**: Each user (project, root, debugger) can have unique passwords
4. **Backward Compatibility**: Temporary fallback to environment variables during migration
5. **Password Validation**: Script fails if passwords are not available
6. **No Password Logging**: Removed password disclosure from logs

### Migration Strategy:

1. **Phase 1a**: Implement the secret reading logic with fallback to environment variables
2. **Phase 1b**: Update docker-compose files to use secrets
3. **Phase 1c**: Generate and deploy unique passwords for each environment
4. **Phase 1d**: Remove environment variable fallbacks once secrets are fully deployed

This approach provides immediate security improvements while maintaining backward compatibility during the transition period.
