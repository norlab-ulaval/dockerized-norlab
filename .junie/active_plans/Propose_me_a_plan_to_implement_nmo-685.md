## Docker Secret Management Best-Practice Implementation Plan
References:
- NMO-685 fix: implement secret best-practice

Based on my analysis of the SSH daemon implementation in your dockerized-norlab project, I've identified several security vulnerabilities in the current approach and propose a comprehensive plan to implement Docker secret management best practices.

### Current Security Issues

1. Hardcoded default password set via `ARG DN_SSH_SERVER_USER_PASSWORD=*****` in `dockerized-norlab-images/core-images/dn-project/project-develop/Dockerfile`
2. Build argument exposure: secrets passed as build args can leak via image history and caches
3. Shared password: the same value is applied to root, project user, and SSH user in `dockerized-norlab-images/container-tools/dn_install_debugging_tools.bash`
4. Password-based auth enabled by default and `PermitRootLogin yes` present in `dockerized-norlab-images/container-tools/dn_install_debugging_tools.bash`
5. No secret rotation mechanism is defined

### Recommended Implementation Plan

#### Phase 1: Immediate Security Improvements

**1.1 Remove Hardcoded Passwords**
```dockerfile
# Replace in Dockerfile
ARG DN_SSH_SERVER_USER_PASSWORD
# Remove default value completely
```

**1.2 Implement Docker Secrets**
- Build-time (in this repo): define BuildKit build secrets in `dockerized-norlab-images/core-images/dn-project/docker-compose.dn-project.build.yaml` and consume them from `dockerized-norlab-images/core-images/dn-project/project-develop/Dockerfile`.
- Runtime (in super project DNA): mount runtime secrets (e.g., authorized_keys) in the app-level compose that actually runs the containers.

Example (build-time compose, BuildKit):
```yaml
# File: dockerized-norlab-images/core-images/dn-project/docker-compose.dn-project.build.yaml
services:
  project-develop-main:
    build:
      context: project-develop
      dockerfile: Dockerfile
      secrets:
        - id: ssh_user_password
          src: ./secrets/ssh_user_password.txt
        - id: ssh_root_password
          src: ./secrets/ssh_root_password.txt
      args:
        # Remove DN_SSH_SERVER_USER_PASSWORD from here
secrets: {}
```

Example (Dockerfile build-time usage with BuildKit):
```dockerfile
# in dockerized-norlab-images/core-images/dn-project/project-develop/Dockerfile
# ... before sourcing dn_install_debugging_tools.bash
RUN --mount=type=secret,id=ssh_user_password,required=1 \
    --mount=type=secret,id=ssh_root_password,required=1 \
    export DN_SSH_SERVER_USER_PASSWORD="$(cat /run/secrets/ssh_user_password)" && \
    export DN_ROOT_PASSWORD="$(cat /run/secrets/ssh_root_password)" && \
    source ./dn_install_debugging_tools.bash
```

Note: For runtime key-based auth, add a secret like `ssh_public_key` in the super project (DNA) compose that runs `project-develop`, and mount it to `/run/secrets/ssh_public_key`.

**1.3 Update Build Process**
- Create a `secrets/` directory (add to `.gitignore`)
- Generate strong, unique passwords for each environment
- Use Docker BuildKit secrets mount for build-time secrets

#### Phase 2: Enhanced Secret Management

**2.1 Implement SSH Key-Based Authentication**
```bash
# Modify dn_install_debugging_tools.bash
function dn::setup_ssh_key_auth() {
  # Disable password authentication
  echo "PasswordAuthentication no" >> /etc/ssh/sshd_config_dockerized_norlab_openssh_server
  echo "PubkeyAuthentication yes" >> /etc/ssh/sshd_config_dockerized_norlab_openssh_server
  
  # Setup authorized_keys from Docker secret
  mkdir -p "/home/${DN_SSH_SERVER_USER}/.ssh"
  if [ -f /run/secrets/ssh_public_key ]; then
    cp /run/secrets/ssh_public_key "/home/${DN_SSH_SERVER_USER}/.ssh/authorized_keys"
    chmod 600 "/home/${DN_SSH_SERVER_USER}/.ssh/authorized_keys"
    chown "${DN_SSH_SERVER_USER}:${DN_SSH_SERVER_USER}" "/home/${DN_SSH_SERVER_USER}/.ssh/authorized_keys"
  fi
}
```

**2.2 Runtime Secret Injection**
```yaml
# Update docker-compose files
services:
  project-develop:
    secrets:
      - source: ssh_public_key
        target: /run/secrets/ssh_public_key
        mode: 0400
    environment:
      - SSH_AUTH_METHOD=key  # Flag to use key-based auth
```

#### Phase 3: Advanced Secret Management

**3.1 External Secret Management Integration**
- **HashiCorp Vault**: For enterprise environments
- **AWS Secrets Manager**: For AWS deployments
- **Azure Key Vault**: For Azure deployments
- **Docker Swarm Secrets**: For orchestrated deployments

**3.2 Secret Rotation Mechanism**
```bash
# Create dn_rotate_secrets.bash
#!/bin/bash
function dn::rotate_ssh_secrets() {
  # Generate new SSH key pair
  ssh-keygen -t ed25519 -f ./secrets/ssh_key_new -N ""
  
  # Update Docker secrets
  docker secret create ssh_public_key_v2 ./secrets/ssh_key_new.pub
  docker service update --secret-rm ssh_public_key --secret-add ssh_public_key_v2 project-develop
  
  # Cleanup old secrets after verification
  docker secret rm ssh_public_key
}
```

#### Phase 4: Security Hardening

**4.1 Update SSH Configuration**
```bash
# Enhanced SSH security in dn_install_debugging_tools.bash
cat > /etc/ssh/sshd_config_dockerized_norlab_openssh_server << EOF
# Security hardening
Protocol 2
Port ${DN_SSH_SERVER_PORT}
PermitRootLogin no
PasswordAuthentication no
PubkeyAuthentication yes
AuthorizedKeysFile .ssh/authorized_keys
PermitEmptyPasswords no
ChallengeResponseAuthentication no
UsePAM yes
X11Forwarding yes
PrintMotd no
AcceptEnv LANG LC_*
Subsystem sftp /usr/lib/openssh/sftp-server
# Rate limiting
MaxAuthTries 3
MaxSessions 2
ClientAliveInterval 300
ClientAliveCountMax 2
EOF
```

**4.2 Implement Logging and Monitoring**
```bash
# Add to dn_entrypoint.init.bash
function dn::setup_ssh_monitoring() {
  # Setup SSH logging
  echo "LogLevel VERBOSE" >> /etc/ssh/sshd_config_dockerized_norlab_openssh_server
  
  # Monitor failed login attempts
  tail -f /var/log/auth.log | grep "Failed password" &
}
```

#### Phase 5: Development Workflow Integration

**5.1 Developer Setup Script**
```bash
#!/bin/bash
# dn_setup_dev_ssh.bash
function dn::setup_developer_ssh() {
  echo "Setting up SSH keys for development..."
  
  # Generate developer SSH key if not exists
  if [ ! -f ~/.ssh/dn_project_key ]; then
    ssh-keygen -t ed25519 -f ~/.ssh/dn_project_key -C "dev@dockerized-norlab"
  fi
  
  # Copy public key to secrets directory
  cp ~/.ssh/dn_project_key.pub ./secrets/ssh_public_key.txt
  
  echo "SSH key setup complete. Use: ssh -i ~/.ssh/dn_project_key ${DN_SSH_SERVER_USER}@localhost -p 2222"
}
```

**5.2 IDE Integration Guide**
- PyCharm: Configure SSH interpreter with private key
- VS Code: Update remote SSH configuration
- CLion: Configure remote toolchain with key authentication

### Implementation Priority

1. **High Priority** (Security Critical):
   - Remove hardcoded passwords
   - Implement Docker secrets for build-time secrets
   - Disable password authentication

2. **Medium Priority** (Enhanced Security):
   - SSH key-based authentication
   - Secret rotation mechanism
   - SSH configuration hardening

3. **Low Priority** (Advanced Features):
   - External secret management integration
   - Comprehensive monitoring and logging

### Migration Strategy

1. **Backward Compatibility**: Maintain password authentication as fallback during transition
2. **Gradual Rollout**: Implement changes in development environment first
3. **Documentation**: Update all documentation and developer guides
4. **Testing**: Comprehensive testing of SSH connectivity in all environments

### Security Benefits

- **Eliminated Secret Exposure**: No more passwords in Docker history or logs
- **Strong Authentication**: SSH key-based authentication is more secure
- **Secret Rotation**: Ability to rotate secrets without rebuilding images
- **Audit Trail**: Better logging and monitoring of SSH access
- **Principle of Least Privilege**: Each user has unique credentials

This plan provides a comprehensive approach to implementing Docker secret management best practices while maintaining the development workflow requirements of your dockerized-norlab project.
