#!/usr/bin/env python3
"""
Process Manager for Yuvaan Robot System
Handles local and remote process lifecycle management with logging
"""

import os
import subprocess
import signal
import time
import logging
from typing import Optional, Dict, Callable
from pathlib import Path
from datetime import datetime
from enum import Enum

logger = logging.getLogger(__name__)


class ProcessState(Enum):
    """Process state enumeration"""
    STOPPED = "stopped"
    STARTING = "starting"
    RUNNING = "running"
    STOPPING = "stopping"
    FAILED = "failed"
    UNKNOWN = "unknown"


class ManagedProcess:
    """Represents a managed process with lifecycle control"""
    
    def __init__(self, name: str, command: str, location: str,
                 log_dir: Path, ssh_manager=None, env: Optional[Dict] = None):
        self.name = name
        self.command = command
        self.location = location
        self.log_dir = log_dir
        self.ssh_manager = ssh_manager
        self.env = env or {}
        
        self.process: Optional[subprocess.Popen] = None
        self.state = ProcessState.STOPPED
        self.start_time: Optional[datetime] = None
        self.restart_count = 0
        
        # Log file paths
        self.log_file = log_dir / f"{name}.log"
        self.stdout_file = log_dir / f"{name}_stdout.log"
        self.stderr_file = log_dir / f"{name}_stderr.log"
        
    def start(self) -> bool:
        """Start the process"""
        if self.state == ProcessState.RUNNING:
            logger.warning(f"Process {self.name} is already running")
            return True
        
        try:
            self.state = ProcessState.STARTING
            logger.info(f"Starting {self.name} on {self.location}...")
            
            # Ensure log directory exists
            self.log_dir.mkdir(parents=True, exist_ok=True)
            
            # Open log files
            stdout_log = open(self.stdout_file, 'a')
            stderr_log = open(self.stderr_file, 'a')
            
            # Log start event
            with open(self.log_file, 'a') as f:
                f.write(f"\n{'='*60}\n")
                f.write(f"[{datetime.now()}] Starting {self.name}\n")
                f.write(f"Command: {self.command}\n")
                f.write(f"Location: {self.location}\n")
                f.write(f"{'='*60}\n\n")
            
            if self.location == "laptop":
                # Run locally
                self.process = subprocess.Popen(
                    self.command,
                    shell=True,
                    stdout=stdout_log,
                    stderr=stderr_log,
                    env={**os.environ, **self.env},
                    preexec_fn=os.setsid  # Create new process group
                )
                
            else:  # jetson
                # Run remotely via SSH
                if not self.ssh_manager:
                    raise RuntimeError("SSH manager not provided for remote execution")
                
                # Prepare remote log file path
                remote_log_file = f"/home/jetson/yuvaan_logs/{self.name}_stdout.log"
                
                # Use SSH manager's execute_command which sources ROS environment
                # Execute in background with logging to remote file
                returncode, stdout, stderr = self.ssh_manager.execute_command(
                    self.command,
                    background=True,
                    env=self.env,
                    log_file=remote_log_file
                )
                
                # For background processes, we don't have a local process object
                # Set a dummy value since we track via SSH
                self.process = subprocess.Popen(['true'])  # Placeholder process
            
            # Close log files
            stdout_log.close()
            stderr_log.close()
            
            # Wait for process to start
            # Remote processes need more time to initialize, especially ROS nodes
            wait_time = 3.0 if self.location == "jetson" else 0.5
            time.sleep(wait_time)
            
            if self.is_running():
                self.state = ProcessState.RUNNING
                self.start_time = datetime.now()
                logger.info(f"✓ {self.name} started successfully")
                return True
            else:
                self.state = ProcessState.FAILED
                logger.error(f"✗ {self.name} failed to start")
                return False
                
        except Exception as e:
            self.state = ProcessState.FAILED
            logger.error(f"Error starting {self.name}: {e}")
            return False
    
    def stop(self, timeout: int = 10) -> bool:
        """Stop the process gracefully"""
        if self.state == ProcessState.STOPPED:
            logger.warning(f"Process {self.name} is already stopped")
            return True
        
        try:
            self.state = ProcessState.STOPPING
            logger.info(f"Stopping {self.name}...")
            
            # Log stop event
            with open(self.log_file, 'a') as f:
                f.write(f"\n[{datetime.now()}] Stopping {self.name}\n")
            
            if self.location == "laptop":
                # Stop local process
                if self.process:
                    try:
                        # Send SIGTERM to process group
                        os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
                        
                        # Wait for process to terminate
                        try:
                            self.process.wait(timeout=timeout)
                        except subprocess.TimeoutExpired:
                            # Force kill if timeout
                            logger.warning(f"Process {self.name} did not stop gracefully, forcing...")
                            os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
                            self.process.wait()
                        
                    except ProcessLookupError:
                        pass  # Process already terminated
                    
            else:  # jetson
                # Stop remote process
                if self.ssh_manager:
                    # Determine search pattern (same logic as is_running)
                    if self.command.startswith("rosrun"):
                        # For rosrun, search by script name
                        parts = self.command.split()
                        search_pattern = parts[2] if len(parts) >= 3 else self.name
                    elif self.command.startswith("rostopic"):
                        # For rostopic, search by command pattern
                        search_term = self.command.split()[0:3]
                        search_pattern = " ".join(search_term) if len(search_term) > 1 else self.command
                    else:
                        search_pattern = self.name
                    
                    # Kill process
                    self.ssh_manager.kill_process(search_pattern, "TERM")
                    
                    # Wait and verify
                    time.sleep(2)
                    if self.ssh_manager.check_process_running(search_pattern):
                        # Force kill if still running
                        logger.warning(f"Remote process {self.name} did not stop gracefully, forcing...")
                        self.ssh_manager.kill_process(search_pattern, "KILL")
            
            self.state = ProcessState.STOPPED
            self.process = None
            logger.info(f"✓ {self.name} stopped")
            return True
            
        except Exception as e:
            logger.error(f"Error stopping {self.name}: {e}")
            self.state = ProcessState.UNKNOWN
            return False
    
    def restart(self) -> bool:
        """Restart the process"""
        logger.info(f"Restarting {self.name}...")
        self.restart_count += 1
        
        if not self.stop():
            logger.error(f"Failed to stop {self.name} for restart")
            return False
        
        time.sleep(1)
        return self.start()
    
    def is_running(self) -> bool:
        """Check if process is currently running"""
        if self.location == "laptop":
            # Check local process
            if self.process:
                return self.process.poll() is None
            return False
            
        else:  # jetson
            # Check remote process
            if self.ssh_manager:
                # Determine search pattern based on command type
                if self.command.startswith("rosrun"):
                    # For rosrun, search by script name only
                    # "rosrun yuvaan_controller merged_control.py" -> search "merged_control.py"
                    parts = self.command.split()
                    if len(parts) >= 3:
                        search_pattern = parts[2]  # The script name
                    else:
                        search_pattern = self.name
                elif self.command.startswith("rostopic"):
                    # For rostopic, search by full command pattern
                    search_term = self.command.split()[0:3]  # ["rostopic", "echo", "/topic"]
                    search_pattern = " ".join(search_term) if len(search_term) > 1 else self.command
                else:
                    # For other commands, search by component name
                    search_pattern = self.name
                
                return self.ssh_manager.check_process_running(search_pattern)
            return False
    
    def get_status(self) -> Dict:
        """Get current process status"""
        is_running = self.is_running()
        
        # Update state based on actual running status
        if is_running and self.state != ProcessState.RUNNING:
            self.state = ProcessState.RUNNING
        elif not is_running and self.state == ProcessState.RUNNING:
            self.state = ProcessState.STOPPED
        
        uptime = None
        if self.start_time and is_running:
            uptime = (datetime.now() - self.start_time).total_seconds()
        
        return {
            "name": self.name,
            "state": self.state.value,
            "location": self.location,
            "is_running": is_running,
            "uptime": uptime,
            "restart_count": self.restart_count,
            "pid": self.process.pid if self.process else None,
            "log_file": str(self.log_file)
        }
    
    def tail_logs(self, lines: int = 50) -> str:
        """Get last N lines from stdout log"""
        try:
            if self.stdout_file.exists():
                with open(self.stdout_file, 'r') as f:
                    return ''.join(f.readlines()[-lines:])
            return ""
        except Exception as e:
            logger.error(f"Error reading logs for {self.name}: {e}")
            return f"Error reading logs: {e}"


class ProcessManager:
    """Manages multiple processes with dependency resolution"""
    
    def __init__(self, log_dir: Path, ssh_manager=None):
        self.log_dir = Path(log_dir).expanduser()
        self.ssh_manager = ssh_manager
        self.processes: Dict[str, ManagedProcess] = {}
        self.dependencies: Dict[str, list] = {}
        
    def add_process(self, name: str, command: str, location: str,
                   dependencies: list = None, env: Dict = None):
        """Add a process to be managed"""
        process = ManagedProcess(
            name=name,
            command=command,
            location=location,
            log_dir=self.log_dir,
            ssh_manager=self.ssh_manager if location == "jetson" else None,
            env=env
        )
        
        self.processes[name] = process
        self.dependencies[name] = dependencies or []
        
        logger.debug(f"Added process {name} with dependencies: {dependencies}")
    
    def start_process(self, name: str, startup_delay: float = 0) -> bool:
        """Start a process and its dependencies"""
        if name not in self.processes:
            logger.error(f"Process {name} not found")
            return False
        
        # Start dependencies first
        for dep in self.dependencies[name]:
            if dep not in self.processes:
                logger.error(f"Dependency {dep} not found for {name}")
                return False
            
            if not self.processes[dep].is_running():
                logger.info(f"Starting dependency {dep} for {name}")
                if not self.start_process(dep):
                    logger.error(f"Failed to start dependency {dep}")
                    return False
        
        # Start the process
        success = self.processes[name].start()
        
        if success and startup_delay > 0:
            logger.debug(f"Waiting {startup_delay}s after starting {name}")
            time.sleep(startup_delay)
        
        return success
    
    def stop_process(self, name: str, cascade: bool = True) -> bool:
        """Stop a process and optionally its dependents"""
        if name not in self.processes:
            logger.error(f"Process {name} not found")
            return False
        
        # If cascade, stop dependent processes first
        if cascade:
            dependents = self._get_dependents(name)
            for dep in dependents:
                logger.info(f"Stopping dependent {dep}")
                self.stop_process(dep, cascade=False)
        
        return self.processes[name].stop()
    
    def restart_process(self, name: str) -> bool:
        """Restart a process"""
        if name not in self.processes:
            logger.error(f"Process {name} not found")
            return False
        
        return self.processes[name].restart()
    
    def get_status(self, name: Optional[str] = None) -> Dict:
        """Get status of one or all processes"""
        if name:
            if name in self.processes:
                return self.processes[name].get_status()
            return {}
        
        return {name: proc.get_status() for name, proc in self.processes.items()}
    
    def stop_all(self):
        """Stop all running processes in reverse dependency order"""
        # Build reverse dependency order
        stop_order = self._get_stop_order()
        
        for name in stop_order:
            if self.processes[name].is_running():
                self.stop_process(name, cascade=False)
    
    def _get_dependents(self, name: str) -> list:
        """Get all processes that depend on the given process"""
        dependents = []
        for proc_name, deps in self.dependencies.items():
            if name in deps:
                dependents.append(proc_name)
        return dependents
    
    def _get_stop_order(self) -> list:
        """Get order to stop processes (reverse of start order)"""
        # Simple topological sort for reverse dependencies
        visited = set()
        order = []
        
        def visit(name):
            if name in visited:
                return
            visited.add(name)
            
            for dependent in self._get_dependents(name):
                visit(dependent)
            
            order.append(name)
        
        for name in self.processes.keys():
            visit(name)
        
        return order


if __name__ == "__main__":
    # Test process manager
    logging.basicConfig(level=logging.INFO)
    
    pm = ProcessManager(log_dir=Path("~/test_logs"))
    
    # Add test processes
    pm.add_process("test1", "sleep 60", "laptop")
    pm.add_process("test2", "sleep 60", "laptop", dependencies=["test1"])
    
    # Start processes
    pm.start_process("test2")
    
    # Check status
    print(pm.get_status())
    
    # Stop all
    time.sleep(2)
    pm.stop_all()
