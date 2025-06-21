## Docker Secret Management Best-Practice Implementation Plan
References:
- NMO-685 fix: implement secret best-practice

Based on my analysis of the SSH daemon implementation in your dockerized-norlab project, I've identified several security vulnerabilities in the current approach and propose a comprehensive plan to implement Docker secret management best practices.

### Current Security Issues

1. **Hardcoded Password**: The SSH password is hardcoded as "lasagne" in the Dockerfile (line 44)
2. **Build Argument Exposure**: Passwords passed as build arguments are visible in Docker image history
3. **Shared Password**: The same password is used for root, project user, and SSH user
4. **Plain Text Logging**: The password is displayed in plain text in the entrypoint script (line 20)
5. **No Secret Rotation**: No mechanism for password rotation or management

### Recommended Implementation Plan

#### Phase 1: Immediate Security Improvements

**1.1 Remove Hardcoded Passwords**
```dockerfile
# Replace in Dockerfile
ARG DN_SSH_SERVER_USER_PASSWORD
# Remove default value completely
```

**1.2 Implement Docker Secrets**
```yaml
# Add to docker-compose.dn-project.build.yaml
secrets:
  ssh_user_password:
    file: ./secrets/ssh_user_password.txt
  ssh_root_password:
    file: ./secrets/ssh_root_password.txt

services:
  project-develop-main:
    secrets:
      - ssh_user_password
      - ssh_root_password
    build:
      args:
        # Remove DN_SSH_SERVER_USER_PASSWORD from here
```

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
