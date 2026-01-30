#!/usr/bin/env python3
"""
SSH Utilities for Yuvaan Robot System Manager
Handles SSH connections, command execution, and SSH key setup for Jetson
"""

import os
import subprocess
import time
import logging
from typing import Optional, Tuple, Dict
from pathlib import Path

logger = logging.getLogger(__name__)


class SSHManager:
    """Manages SSH connections and remote command execution"""
    
    def __init__(self, host: str, user: str, password: Optional[str] = None):
        self.host = host
        self.user = user
        self.password = password
        self.workspace_path = None  # Can be set later if needed
        self.ssh_key_path = Path.home() / ".ssh" / "id_rsa"
        self.connection_tested = False
        
    def setup_ssh_keys(self) -> bool:
        """
        Set up SSH keys for passwordless authentication
        Returns True if keys are already set up or successfully created
        """
        try:
            # Check if SSH key exists
            if not self.ssh_key_path.exists():
                logger.info("Generating SSH key pair...")
                result = subprocess.run(
                    ["ssh-keygen", "-t", "rsa", "-b", "4096", "-f", 
                     str(self.ssh_key_path), "-N", ""],
                    capture_output=True,
                    text=True
                )
                if result.returncode != 0:
                    logger.error(f"Failed to generate SSH key: {result.stderr}")
                    return False
                logger.info("SSH key pair generated successfully")
            
            # Test if passwordless SSH already works
            if self.test_passwordless_connection():
                logger.info("Passwordless SSH already configured")
                return True
            
            # Copy SSH key to remote host
            logger.info(f"Setting up passwordless SSH to {self.user}@{self.host}...")
            
            # Use ssh-copy-id if available
            if self._command_exists("ssh-copy-id"):
                cmd = f"sshpass -p '{self.password}' ssh-copy-id -i {self.ssh_key_path}.pub {self.user}@{self.host}"
                result = subprocess.run(
                    cmd,
                    shell=True,
                    capture_output=True,
                    text=True
                )
                if result.returncode == 0:
                    logger.info("SSH key copied successfully")
                    return True
                else:
                    logger.warning(f"ssh-copy-id failed: {result.stderr}")
            
            # Fallback: manually copy key
            return self._manual_copy_ssh_key()
            
        except Exception as e:
            logger.error(f"Error setting up SSH keys: {e}")
            return False
    
    def _command_exists(self, command: str) -> bool:
        """Check if a command exists in PATH"""
        return subprocess.run(
            ["which", command],
            capture_output=True
        ).returncode == 0
    
    def _manual_copy_ssh_key(self) -> bool:
        """Manually copy SSH key to remote host"""
        try:
            # Read public key
            with open(f"{self.ssh_key_path}.pub", 'r') as f:
                pub_key = f.read().strip()
            
            # Create command to append key to authorized_keys
            remote_cmd = (
                f"mkdir -p ~/.ssh && "
                f"chmod 700 ~/.ssh && "
                f"echo '{pub_key}' >> ~/.ssh/authorized_keys && "
                f"chmod 600 ~/.ssh/authorized_keys"
            )
            
            # Execute using sshpass
            cmd = f"sshpass -p '{self.password}' ssh {self.user}@{self.host} '{remote_cmd}'"
            result = subprocess.run(
                cmd,
                shell=True,
                capture_output=True,
                text=True
            )
            
            if result.returncode == 0:
                logger.info("SSH key manually copied successfully")
                return True
            else:
                logger.error(f"Failed to copy SSH key manually: {result.stderr}")
                return False
                
        except Exception as e:
            logger.error(f"Error in manual SSH key copy: {e}")
            return False
    
    def test_passwordless_connection(self) -> bool:
        """Test if passwordless SSH connection works"""
        try:
            result = subprocess.run(
                ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5",
                 f"{self.user}@{self.host}", "echo", "test"],
                capture_output=True,
                text=True,
                timeout=10
            )
            self.connection_tested = result.returncode == 0
            return self.connection_tested
        except Exception as e:
            logger.error(f"Error testing SSH connection: {e}")
            return False
    
    def execute_command(self, command: str, background: bool = False, 
                       env: Optional[Dict[str, str]] = None, log_file: Optional[str] = None) -> Tuple[int, str, str]:
        """
        Execute a command on the remote host via SSH
        
        Args:
            command: Command to execute
            background: If True, run command in background and return immediately
            env: Environment variables to set for the command
            log_file: Optional log file path (on remote host) for background command output
            
        Returns:
            Tuple of (return_code, stdout, stderr)
        """
        # Source ROS environment before command
        # This is critical for non-interactive SSH sessions
        ros_setup = "source /opt/ros/noetic/setup.bash"
        workspace_setup = f"source {self.workspace_path}/devel/setup.bash" if self.workspace_path else ""
        
        # Build full command with environment sourcing
        if workspace_setup:
            env_setup = f'bash -c "{ros_setup} && {workspace_setup} && {command}"'
        else:
            env_setup = f'bash -c "{ros_setup} && {command}"'
        
        # Build environment prefix if provided
        if env:
            env_str = " ".join([f"{k}={v}" for k, v in env.items()])
            env_setup = f"{env_str} {env_setup}"
        
        # Build SSH command
        if background:
            # Run in background with nohup
            if log_file:
                # Redirect to log file if provided
                full_command = f"nohup {env_setup} >> {log_file} 2>&1 &"
            else:
                # Redirect to /dev/null if no log file
                full_command = f"nohup {env_setup} > /dev/null 2>&1 &"
        else:
            full_command = env_setup
        
        ssh_cmd = [
            "ssh",
            "-o", "StrictHostKeyChecking=no",
            f"{self.user}@{self.host}",
            full_command
        ]
        
        try:
            if background:
                # For background commands, just start and return
                subprocess.Popen(ssh_cmd)
                return (0, "", "")
            else:
                # For foreground commands, wait for completion
                result = subprocess.run(
                    ssh_cmd,
                    capture_output=True,
                    text=True,
                    timeout=30
                )
                return (result.returncode, result.stdout, result.stderr)
                
        except subprocess.TimeoutExpired:
            logger.error(f"Command timed out: {command}")
            return (-1, "", "Command timed out")
        except Exception as e:
            logger.error(f"Error executing command: {e}")
            return (-1, "", str(e))
    
    def execute_command_stream(self, command: str, callback=None):
        """
        Execute a command and stream output in real-time
        
        Args:
            command: Command to execute
            callback: Optional callback function called with each line of output
            
        Returns:
            subprocess.Popen object
        """
        ssh_cmd = [
            "ssh",
            "-o", "StrictHostKeyChecking=no",
            f"{self.user}@{self.host}",
            command
        ]
        
        try:
            process = subprocess.Popen(
                ssh_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1
            )
            
            if callback:
                # Stream output through callback
                for line in process.stdout:
                    callback(line.rstrip())
            
            return process
            
        except Exception as e:
            logger.error(f"Error executing streaming command: {e}")
            return None
    
    def execute_system_command(self, command: str) -> Tuple[int, str, str]:
        """
        Execute a system command that doesn't need ROS environment
        Used for pgrep, pkill, etc.
        """
        ssh_cmd = [
            "ssh",
            "-o", "StrictHostKeyChecking=no",
            f"{self.user}@{self.host}",
            command
        ]
        
        try:
            result = subprocess.run(
                ssh_cmd,
                capture_output=True,
                text=True,
                timeout=10
            )
            return (result.returncode, result.stdout, result.stderr)
        except subprocess.TimeoutExpired:
            logger.error(f"System command timed out: {command}")
            return (-1, "", "Command timed out")
        except Exception as e:
            logger.error(f"Error executing system command: {e}")
            return (-1, "", str(e))
    
    def check_process_running(self, process_name: str) -> bool:
        """Check if a process is running on the remote host"""
        returncode, stdout, stderr = self.execute_system_command(
            f"pgrep -f '{process_name}'"
        )
        return returncode == 0 and len(stdout.strip()) > 0
    
    def kill_process(self, process_name: str, signal: str = "TERM") -> bool:
        """Kill a process on the remote host"""
        returncode, stdout, stderr = self.execute_system_command(
            f"pkill -{signal} -f '{process_name}'"
        )
        return returncode == 0
    
    def ping(self, count: int = 1, timeout: int = 2) -> bool:
        """Ping the remote host to check connectivity"""
        try:
            result = subprocess.run(
                ["ping", "-c", str(count), "-W", str(timeout), self.host],
                capture_output=True,
                timeout=timeout + 1
            )
            return result.returncode == 0
        except Exception:
            return False


def install_sshpass_if_needed():
    """Install sshpass if not available (needed for initial SSH key setup)"""
    try:
        # Check if sshpass is installed
        result = subprocess.run(
            ["which", "sshpass"],
            capture_output=True
        )
        
        if result.returncode == 0:
            return True
        
        logger.info("sshpass not found, attempting to install...")
        
        # Try to install sshpass
        install_cmd = None
        if os.path.exists("/usr/bin/apt"):
            install_cmd = "sudo apt update && sudo apt install -y sshpass"
        elif os.path.exists("/usr/bin/yum"):
            install_cmd = "sudo yum install -y sshpass"
        elif os.path.exists("/usr/bin/pacman"):
            install_cmd = "sudo pacman -S --noconfirm sshpass"
        
        if install_cmd:
            logger.info(f"Running: {install_cmd}")
            result = subprocess.run(
                install_cmd,
                shell=True,
                capture_output=True,
                text=True
            )
            
            if result.returncode == 0:
                logger.info("sshpass installed successfully")
                return True
            else:
                logger.warning(f"Failed to install sshpass: {result.stderr}")
                logger.warning("You may need to manually install sshpass for initial SSH setup")
                return False
        else:
            logger.warning("Could not determine package manager to install sshpass")
            return False
            
    except Exception as e:
        logger.error(f"Error installing sshpass: {e}")
        return False


if __name__ == "__main__":
    # Test SSH manager
    logging.basicConfig(level=logging.INFO)
    
    ssh = SSHManager("192.168.2.101", "jetson", "jetson")
    
    print("Testing SSH connection...")
    if ssh.test_passwordless_connection():
        print("✓ Passwordless SSH already configured")
    else:
        print("✗ Passwordless SSH not configured, setting up...")
        install_sshpass_if_needed()
        if ssh.setup_ssh_keys():
            print("✓ SSH keys set up successfully")
        else:
            print("✗ Failed to set up SSH keys")
    
    print("\nTesting ping...")
    if ssh.ping():
        print("✓ Host is reachable")
    else:
        print("✗ Host is not reachable")
    
    print("\nTesting command execution...")
    code, out, err = ssh.execute_command("echo 'Hello from Jetson'")
    if code == 0:
        print(f"✓ Command executed: {out.strip()}")
    else:
        print(f"✗ Command failed: {err}")
