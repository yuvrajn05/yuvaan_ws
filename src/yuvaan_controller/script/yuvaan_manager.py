#!/usr/bin/env python3
"""
Yuvaan Robot System Manager
Main control script for managing all robot components with lifecycle control,
health monitoring, logging, and rollback capabilities
"""

import os
import sys
import yaml
import logging
import argparse
import subprocess
from pathlib import Path
from typing import Dict, List, Optional
from datetime import datetime
import time

# Add script directory to path
script_dir = Path(__file__).parent
sys.path.append(str(script_dir))

from ssh_utils import SSHManager, install_sshpass_if_needed
from process_manager import ProcessManager
from state_manager import StateManager

# Color codes for terminal output
class Colors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


class YuvaanSystemManager:
    """Main system manager for Yuvaan robot"""
    
    def __init__(self, config_file: Path, execution_mode: str = "remote", auto_monitor: bool = False):
        self.config_file = Path(config_file)
        self.config = self._load_config()
        self.execution_mode = execution_mode
        self.auto_monitor = auto_monitor
        self.monitor_session = "yuvaan_auto_monitor"
        self.monitor_panes = {}  # Track pane IDs by component name
        
        # Setup logging
        self.log_dir = Path(self.config['logging']['log_dir']).expanduser()
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self._setup_logging()
        
        # Initialize managers
        self.ssh_manager = None
        if execution_mode == "remote":
            self.ssh_manager = SSHManager(
                host=self.config['network']['jetson_ip'],
                user=self.config['network']['jetson_user'],
                password=self.config['network']['jetson_password']
            )
            # Set workspace path for ROS environment sourcing
            if 'jetson_workspace' in self.config['network']:
                self.ssh_manager.workspace_path = self.config['network']['jetson_workspace']
        
        self.process_manager = ProcessManager(
            log_dir=self.log_dir,
            ssh_manager=self.ssh_manager
        )
        
        self.state_manager = StateManager(
            state_dir=self.log_dir / "state"
        )
        
        self.current_group: Optional[str] = None
        
        # Load components into process manager
        self._load_components()
        
        # Setup monitoring session if enabled
        if self.auto_monitor:
            self._setup_monitor_session()
        
        logging.info(f"Yuvaan System Manager initialized in {execution_mode} mode" + 
                    (" with auto-monitoring" if auto_monitor else ""))
    
    def _load_config(self) -> Dict:
        """Load configuration from YAML file"""
        try:
            with open(self.config_file, 'r') as f:
                return yaml.safe_load(f)
        except Exception as e:
            print(f"{Colors.FAIL}Error loading configuration: {e}{Colors.ENDC}")
            sys.exit(1)
    
    def _setup_logging(self):
        """Setup logging infrastructure"""
        # Create formatters and handlers
        log_format = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
        
        # Console handler
        console_handler = logging.StreamHandler()
        console_handler.setLevel(getattr(logging, self.config['logging']['console_level']))
        console_handler.setFormatter(log_format)
        
        # File handler
        file_handler = logging.FileHandler(
            self.log_dir / 'yuvaan_manager.log'
        )
        file_handler.setLevel(getattr(logging, self.config['logging']['file_level']))
        file_handler.setFormatter(log_format)
        
        # Setup root logger
        root_logger = logging.getLogger()
        root_logger.setLevel(logging.DEBUG)
        root_logger.addHandler(console_handler)
        root_logger.addHandler(file_handler)
    
    def _load_components(self):
        """Load all components from config into process manager"""
        for comp in self.config['components']:
            # Determine execution location based on mode
            location = comp['location']
            if self.execution_mode == "local":
                # Override to run locally
                overrides = self.config.get('execution_modes', {}).get('local', {}).get('location_override', {})
                location = overrides.get(location, location)
            
            self.process_manager.add_process(
                name=comp['name'],
                command=comp['command'],
                location=location,
                dependencies=comp.get('dependencies', [])
            )
    
    def _setup_monitor_session(self):
        """Setup tmux session for auto-monitoring"""
        try:
            # Check if tmux is available
            result = subprocess.run(["which", "tmux"], capture_output=True)
            if result.returncode != 0:
                logging.warning("tmux not found, auto-monitoring disabled")
                self.auto_monitor = False
                return
            
            # Kill existing session if it exists
            subprocess.run(["tmux", "kill-session", "-t", self.monitor_session],
                         capture_output=True)
            
            # Create new session (detached) with persistent command
            subprocess.run([
                "tmux", "new-session", "-d", "-s", self.monitor_session,
                "-n", "Monitor",
                "bash", "-c",
                "echo '======================================'; "
                "echo '  Yuvaan Auto-Monitor Session'; "
                "echo '  Components will appear here as they start'; "
                "echo '======================================'; "
                "echo ''; "
                "echo 'Waiting for components...'; "
                "sleep infinity"
            ], check=True)
            
            logging.info(f"Created monitoring session: {self.monitor_session}")
            
        except Exception as e:
            logging.error(f"Failed to setup monitoring session: {e}")
            self.auto_monitor = False
    
    def _get_pane_id_by_title(self, title: str) -> Optional[str]:
        """Get pane ID by its title"""
        try:
            result = subprocess.run([
                "tmux", "list-panes", "-t", self.monitor_session,
                "-F", "#{pane_id}:#{pane_title}"
            ], capture_output=True, text=True)
            
            for line in result.stdout.strip().split('\n'):
                if ':' in line:
                    pane_id, pane_title = line.split(':', 1)
                    if pane_title == title:
                        return pane_id
            return None
        except Exception as e:
            logging.error(f"Failed to get pane ID for {title}: {e}")
            return None
    
    def _add_component_monitor(self, name: str):
        """Add a monitoring pane for a component - runs command directly in pane"""
        if not self.auto_monitor:
            return
        
        # Remove existing pane if it exists
        self._remove_component_monitor(name)
        
        try:
            # Get component information
            process_info = self.process_manager.processes.get(name)
            if not process_info:
                logging.error(f"Component {name} not found in process manager")
                return
            
            # Build the command to run in the pane
            if process_info.location == "jetson":
                # For remote components, SSH and run command directly
                # Source ROS environment and execute command
                ros_setup = "source /opt/ros/noetic/setup.bash"
                workspace_setup = f"source {self.ssh_manager.workspace_path}/devel/setup.bash" if self.ssh_manager and self.ssh_manager.workspace_path else ""
                
                if workspace_setup:
                    remote_cmd = f"{ros_setup} && {workspace_setup} && {process_info.command}"
                else:
                    remote_cmd = f"{ros_setup} && {process_info.command}"
                
                # Create SSH command that runs the component
                pane_cmd = f"ssh -t {self.ssh_manager.user}@{self.ssh_manager.host} 'bash -c \"{remote_cmd}\"'"
                
            else:
                # For local components, run command directly with ROS environment
                # The command should already be properly formatted
                pane_cmd = f"bash -c 'source /opt/ros/noetic/setup.bash && cd {Path.home()}/yuvaan_ws && source devel/setup.bash && {process_info.command}'"
            
            # Create new tmux pane with the command running directly
            subprocess.run([
                "tmux", "split-window", "-t", self.monitor_session,
                "bash", "-c", pane_cmd
            ], capture_output=True)
            
            # Set pane title
            subprocess.run([
                "tmux", "select-pane", "-t", self.monitor_session,
                "-T", name.upper()
            ], capture_output=True)
            
            # Get and store the pane ID
            pane_id = self._get_pane_id_by_title(name.upper())
            if pane_id:
                self.monitor_panes[name] = pane_id
            
            # Rebalance layout
            subprocess.run([
                "tmux", "select-layout", "-t", self.monitor_session, "tiled"
            ], capture_output=True)
            
            logging.debug(f"Added monitor pane for {name} (pane: {pane_id}) - running live command")
            
        except Exception as e:
            logging.error(f"Failed to add monitor for {name}: {e}")
    
    def _remove_component_monitor(self, name: str):
        """Remove a monitoring pane for a component"""
        if not self.auto_monitor:
            return
        
        try:
            # Try to find pane by title
            pane_id = self._get_pane_id_by_title(name.upper())
            
            if pane_id:
                # Kill the pane - use correct target format
                result = subprocess.run([
                    "tmux", "kill-pane", "-t", pane_id
                ], capture_output=True, text=True)
                
                if result.returncode != 0:
                    logging.error(f"Failed to kill pane {pane_id}: {result.stderr}")
                
                # Remove from tracking
                if name in self.monitor_panes:
                    del self.monitor_panes[name]
                
                # Rebalance layout (give it a moment for pane to close)
                time.sleep(0.1)
                subprocess.run([
                    "tmux", "select-layout", "-t", self.monitor_session, "tiled"
                ], capture_output=True)
                
                logging.debug(f"Removed monitor pane for {name}")
            
        except Exception as e:
            logging.debug(f"Failed to remove monitor for {name}: {e}")
    
    def setup_ssh(self) -> bool:
        """Setup SSH connection and keys"""
        if self.execution_mode != "remote":
            logging.info("Running in local mode, skipping SSH setup")
            return True
        
        print(f"\n{Colors.HEADER}{'='*60}")
        print("SSH Setup")
        print(f"{'='*60}{Colors.ENDC}\n")
        
        # Check if sshpass is available
        install_sshpass_if_needed()
        
        # Test connection
        print("Testing network connectivity...")
        if not self.ssh_manager.ping():
            print(f"{Colors.FAIL}✗ Cannot reach Jetson at {self.ssh_manager.host}{Colors.ENDC}")
            print("Please check:")
            print("  1. Jetson is powered on")
            print("  2. Network connection is active")
            print("  3. IP address is correct")
            return False
        
        print(f"{Colors.OKGREEN}✓ Jetson is reachable{Colors.ENDC}")
        
        # Test SSH
        print("\nTesting SSH connection...")
        if self.ssh_manager.test_passwordless_connection():
            print(f"{Colors.OKGREEN}✓ Passwordless SSH already configured{Colors.ENDC}")
            return True
        
        print(f"{Colors.WARNING}Passwordless SSH not configured{Colors.ENDC}")
        print("Setting up SSH keys for passwordless authentication...")
        
        if self.ssh_manager.setup_ssh_keys():
            print(f"{Colors.OKGREEN}✓ SSH keys configured successfully{Colors.ENDC}")
            return True
        else:
            print(f"{Colors.FAIL}✗ Failed to configure SSH keys{Colors.ENDC}")
            print("You may need to manually copy your SSH key to the Jetson")
            return False
    
    def start_component(self, name: str) -> bool:
        """Start a single component"""
        if name not in [c['name'] for c in self.config['components']]:
            logging.error(f"Component {name} not found")
            return False
        
        # Get component config
        comp_config = next(c for c in self.config['components'] if c['name'] == name)
        startup_delay = comp_config.get('startup_delay', 0)
        
        print(f"Starting {Colors.BOLD}{name}{Colors.ENDC}...")
        
        if self.auto_monitor:
            # When monitoring is enabled, run directly in tmux pane (true real-time)
            self._add_component_monitor(name)
            # Give it a moment to start
            time.sleep(startup_delay if startup_delay > 0 else 1)
            # Check if pane still exists (component didn't crash immediately)
            if name in self.monitor_panes:
                print(f"{Colors.OKGREEN}✓ {name} started in monitoring pane{Colors.ENDC}")
                return True
            else:
                print(f"{Colors.FAIL}✗ Failed to start {name}{Colors.ENDC}")
                return False
        else:
            # When monitoring is disabled, use traditional process manager
            success = self.process_manager.start_process(name, startup_delay=startup_delay)
            
            if success:
                print(f"{Colors.OKGREEN}✓ {name} started{Colors.ENDC}")
            else:
                print(f"{Colors.FAIL}✗ Failed to start {name}{Colors.ENDC}")
            
            return success
    
    def stop_component(self, name: str, cascade: bool = True) -> bool:
        """Stop a single component"""
        print(f"Stopping {Colors.BOLD}{name}{Colors.ENDC}...")
        
        if self.auto_monitor:
            # When monitoring is enabled, kill the tmux pane
            self._remove_component_monitor(name)
            print(f"{Colors.OKGREEN}✓ {name} stopped{Colors.ENDC}")
            return True
        else:
            # When monitoring is disabled, use traditional process manager
            success = self.process_manager.stop_process(name, cascade=cascade)
            
            if success:
                print(f"{Colors.OKGREEN}✓ {name} stopped{Colors.ENDC}")
            else:
                print(f"{Colors.FAIL}✗ Failed to stop {name}{Colors.ENDC}")
            
            return success
    
    def restart_component(self, name: str) -> bool:
        """Restart a single component"""
        print(f"Restarting {Colors.BOLD}{name}{Colors.ENDC}...")
        success = self.process_manager.restart_process(name)
        
        if success:
            print(f"{Colors.OKGREEN}✓ {name} restarted{Colors.ENDC}")
            # Recreate monitoring pane if auto-monitor enabled
            if self.auto_monitor:
                self._add_component_monitor(name)
        else:
            print(f"{Colors.FAIL}✗ Failed to restart {name}{Colors.ENDC}")
        
        return success
    
    def start_group(self, group_name: str) -> bool:
        """Start all components in a group"""
        if group_name not in self.config['groups']:
            logging.error(f"Group {group_name} not found")
            return False
        
        group = self.config['groups'][group_name]
        components = group['components']
        
        print(f"\n{Colors.HEADER}{'='*60}")
        print(f"Starting Group: {group_name}")
        print(f"Description: {group['description']}")
        print(f"{'='*60}{Colors.ENDC}\n")
        
        success = True
        for comp_name in components:
            if not self.start_component(comp_name):
                success = False
                logging.error(f"Failed to start component {comp_name} in group {group_name}")
        
        if success:
            self.current_group = group_name
            self.state_manager.save_state(
                self.process_manager,
                mode=self.execution_mode,
                active_group=group_name
            )
        
        return success
    
    def stop_group(self, group_name: str) -> bool:
        """Stop all components in a group"""
        if group_name not in self.config['groups']:
            logging.error(f"Group {group_name} not found")
            return False
        
        group = self.config['groups'][group_name]
        components = group['components']
        
        print(f"\n{Colors.HEADER}Stopping Group: {group_name}{Colors.ENDC}\n")
        
        # Stop in reverse order
        for comp_name in reversed(components):
            self.stop_component(comp_name, cascade=False)
        
        if self.current_group == group_name:
            self.current_group = None
        
        return True
    
    def upgrade_group(self) -> bool:
        """Upgrade to the next group in progression"""
        progression = self.config.get('group_progression', [])
        
        if not self.current_group:
            # Start with first group
            return self.start_group(progression[0])
        
        if self.current_group not in progression:
            print(f"{Colors.WARNING}Current group not in progression{Colors.ENDC}")
            return False
        
        current_idx = progression.index(self.current_group)
        
        if current_idx >= len(progression) - 1:
            print(f"{Colors.WARNING}Already at the highest group level{Colors.ENDC}")
            return False
        
        next_group = progression[current_idx + 1]
        
        print(f"\n{Colors.OKCYAN}Upgrading from {self.current_group} to {next_group}{Colors.ENDC}")
        
        # Get components to add (difference between groups)
        current_comps = set(self.config['groups'][self.current_group]['components'])
        next_comps = set(self.config['groups'][next_group]['components'])
        new_comps = next_comps - current_comps
        
        # Start new components
        for comp_name in new_comps:
            if not self.start_component(comp_name):
                logging.error(f"Failed to start new component {comp_name}")
                return False
        
        self.current_group = next_group
        self.state_manager.save_state(
            self.process_manager,
            mode=self.execution_mode,
            active_group=next_group
        )
        
        return True
    
    def show_status(self):
        """Display system status"""
        print(f"\n{Colors.HEADER}{'='*70}")
        print(f"  Yuvaan Robot System Status")
        print(f"{'='*70}{Colors.ENDC}")
        
        print(f"\n{Colors.BOLD}Mode:{Colors.ENDC} {self.execution_mode}")
        print(f"{Colors.BOLD}Active Group:{Colors.ENDC} {self.current_group or 'None'}")
        
        if self.execution_mode == "remote":
            ping_ok = self.ssh_manager.ping()
            status_icon = f"{Colors.OKGREEN}✓{Colors.ENDC}" if ping_ok else f"{Colors.FAIL}✗{Colors.ENDC}"
            print(f"{Colors.BOLD}Jetson Connectivity:{Colors.ENDC} {status_icon}")
        
        print(f"\n{Colors.BOLD}Components:{Colors.ENDC}")
        print(f"{'─'*70}")
        
        # Header
        print(f"{Colors.BOLD}{'Name':<20} {'Status':<12} {'Location':<10} {'Uptime':<15}{Colors.ENDC}")
        print(f"{'─'*70}")
        
        # Get status of all components
        status = self.process_manager.get_status()
        
        for name in sorted(status.keys()):
            comp_status = status[name]
            is_running = comp_status['is_running']
            
            # Status icon and color
            if is_running:
                status_str = f"{Colors.OKGREEN}● Running{Colors.ENDC}"
            else:
                status_str = f"{Colors.FAIL}○ Stopped{Colors.ENDC}"
            
            # Uptime
            uptime = comp_status.get('uptime')
            if uptime:
                uptime_str = self._format_uptime(uptime)
            else:
                uptime_str = "-"
            
            print(f"{name:<20} {status_str:<23} {comp_status['location']:<10} {uptime_str:<15}")
        
        print(f"{'─'*70}\n")
    
    def _format_uptime(self, seconds: float) -> str:
        """Format uptime in human-readable format"""
        if seconds < 60:
            return f"{int(seconds)}s"
        elif seconds < 3600:
            return f"{int(seconds/60)}m {int(seconds%60)}s"
        else:
            hours = int(seconds / 3600)
            minutes = int((seconds % 3600) / 60)
            return f"{hours}h {minutes}m"
    
    def show_logs(self, component: str, lines: int = 50):
        """Show recent logs for a component"""
        logs = self.process_manager.processes[component].tail_logs(lines)
        
        print(f"\n{Colors.HEADER}{'='*70}")
        print(f"  Logs for {component} (last {lines} lines)")
        print(f"{'='*70}{Colors.ENDC}\n")
        
        print(logs)
    
    def interactive_menu(self):
        """Display interactive menu for system control"""
        while True:
            print(f"\n{Colors.HEADER}{'='*70}")
            print(f"  Yuvaan Robot System Manager - Interactive Menu")
            print(f"{'='*70}{Colors.ENDC}\n")
            
            print(f"{Colors.BOLD}System Control:{Colors.ENDC}")
            print("  1. Show System Status")
            print("  2. Start Component")
            print("  3. Stop Component")
            print("  4. Restart Component")
            print("")
            print(f"{Colors.BOLD}Group Management:{Colors.ENDC}")
            print("  5. Start Group")
            print("  6. Stop Current Group")
            print("  7. Upgrade to Next Group")
            print("")
            print(f"{Colors.BOLD}Monitoring:{Colors.ENDC}")
            print("  8. View Component Logs")
            print("  9. Stream Component Logs (Live)")
            print("  10. Monitor Ping")
            print("")
            print(f"{Colors.BOLD}State Management:{Colors.ENDC}")
            print("  11. Save Current State as Preset")
            print("  12. Load Preset")
            print("  13. Rollback to Previous State")
            print("")
            print("  0. Exit")
            
            choice = input(f"\n{Colors.OKCYAN}Enter choice: {Colors.ENDC}").strip()
            
            if choice == "0":
                print("\nExiting...")
                break
            elif choice == "1":
                self.show_status()
            elif choice == "2":
                self._menu_start_component()
            elif choice == "3":
                self._menu_stop_component()
            elif choice == "4":
                self._menu_restart_component()
            elif choice == "5":
                self._menu_start_group()
            elif choice == "6":
                if self.current_group:
                    self.stop_group(self.current_group)
                else:
                    print(f"{Colors.WARNING}No active group{Colors.ENDC}")
            elif choice == "7":
                self.upgrade_group()
            elif choice == "8":
                self._menu_view_logs()
            elif choice == "9":
                self._menu_stream_logs()
            elif choice == "10":
                self._menu_monitor_ping()
            elif choice == "11":
                self._menu_save_preset()
            elif choice == "12":
                self._menu_load_preset()
            elif choice == "13":
                self._menu_rollback()
            else:
                print(f"{Colors.FAIL}Invalid choice{Colors.ENDC}")
            
            input(f"\n{Colors.OKCYAN}Press Enter to continue...{Colors.ENDC}")
    
    def _menu_start_component(self):
        """Menu for starting a component"""
        components = [c['name'] for c in self.config['components']]
        
        print(f"\n{Colors.BOLD}Available Components:{Colors.ENDC}")
        for i, name in enumerate(components, 1):
            print(f"  {i}. {name}")
        
        choice = input(f"\n{Colors.OKCYAN}Enter component number or name: {Colors.ENDC}").strip()
        
        if choice.isdigit():
            idx = int(choice) - 1
            if 0 <= idx < len(components):
                self.start_component(components[idx])
            else:
                print(f"{Colors.FAIL}Invalid number{Colors.ENDC}")
        elif choice in components:
            self.start_component(choice)
        else:
            print(f"{Colors.FAIL}Component not found{Colors.ENDC}")
    
    def _menu_stop_component(self):
        """Menu for stopping a component"""
        status = self.process_manager.get_status()
        running = [name for name, s in status.items() if s['is_running']]
        
        if not running:
            print(f"{Colors.WARNING}No components are running{Colors.ENDC}")
            return
        
        print(f"\n{Colors.BOLD}Running Components:{Colors.ENDC}")
        for i, name in enumerate(running, 1):
            print(f"  {i}. {name}")
        
        choice = input(f"\n{Colors.OKCYAN}Enter component number or name: {Colors.ENDC}").strip()
        
        if choice.isdigit():
            idx = int(choice) - 1
            if 0 <= idx < len(running):
                self.stop_component(running[idx])
            else:
                print(f"{Colors.FAIL}Invalid number{Colors.ENDC}")
        elif choice in running:
            self.stop_component(choice)
        else:
            print(f"{Colors.FAIL}Component not found{Colors.ENDC}")
    
    def _menu_restart_component(self):
        """Menu for restarting a component"""
        components = [c['name'] for c in self.config['components']]
        
        print(f"\n{Colors.BOLD}Components:{Colors.ENDC}")
        for i, name in enumerate(components, 1):
            status = self.process_manager.get_status(name)
            icon = "●" if status.get('is_running') else "○"
            print(f"  {i}. {icon} {name}")
        
        choice = input(f"\n{Colors.OKCYAN}Enter component number or name: {Colors.ENDC}").strip()
        
        if choice.isdigit():
            idx = int(choice) - 1
            if 0 <= idx < len(components):
                self.restart_component(components[idx])
            else:
                print(f"{Colors.FAIL}Invalid number{Colors.ENDC}")
        elif choice in components:
            self.restart_component(choice)
        else:
            print(f"{Colors.FAIL}Component not found{Colors.ENDC}")
    
    def _menu_start_group(self):
        """Menu for starting a group"""
        groups = list(self.config['groups'].keys())
        
        print(f"\n{Colors.BOLD}Available Groups:{Colors.ENDC}")
        for i, name in enumerate(groups, 1):
            desc = self.config['groups'][name]['description']
            active = " (ACTIVE)" if name == self.current_group else ""
            print(f"  {i}. {name}{active}")
            print(f"      {desc}")
        
        choice = input(f"\n{Colors.OKCYAN}Enter group number or name: {Colors.ENDC}").strip()
        
        if choice.isdigit():
            idx = int(choice) - 1
            if 0 <= idx < len(groups):
                self.start_group(groups[idx])
            else:
                print(f"{Colors.FAIL}Invalid number{Colors.ENDC}")
        elif choice in groups:
            self.start_group(choice)
        else:
            print(f"{Colors.FAIL}Group not found{Colors.ENDC}")
    
    def _menu_view_logs(self):
        """Menu for viewing logs"""
        components = [c['name'] for c in self.config['components']]
        
        print(f"\n{Colors.BOLD}Select Component:{Colors.ENDC}")
        for i, name in enumerate(components, 1):
            print(f"  {i}. {name}")
        
        choice = input(f"\n{Colors.OKCYAN}Enter component number or name: {Colors.ENDC}").strip()
        
        component = None
        if choice.isdigit():
            idx = int(choice) - 1
            if 0 <= idx < len(components):
                component = components[idx]
        elif choice in components:
            component = choice
        
        if component:
            lines = input(f"{Colors.OKCYAN}Number of lines (default 50): {Colors.ENDC}").strip()
            lines = int(lines) if lines.isdigit() else 50
            self.show_logs(component, lines)
        else:
            print(f"{Colors.FAIL}Component not found{Colors.ENDC}")
    
    def _menu_stream_logs(self):
        """Menu for streaming logs in real-time"""
        status = self.process_manager.get_status()
        running = [name for name, s in status.items() if s['is_running']]
        
        if not running:
            print(f"{Colors.WARNING}No components are running{Colors.ENDC}")
            return
        
        print(f"\n{Colors.BOLD}Select Component to Stream:{Colors.ENDC}")
        for i, name in enumerate(running, 1):
            print(f"  {i}. {name}")
        
        choice = input(f"\n{Colors.OKCYAN}Enter component number or name: {Colors.ENDC}").strip()
        
        component = None
        if choice.isdigit():
            idx = int(choice) - 1
            if 0 <= idx < len(running):
                component = running[idx]
        elif choice in running:
            component = choice
        
        if component:
            log_file = self.process_manager.processes[component].stdout_file
            
            print(f"\n{Colors.HEADER}{'='*70}")
            print(f"  Live Stream: {component}")
            print(f"{'='*70}{Colors.ENDC}")
            print(f"{Colors.WARNING}Press Ctrl+C to stop streaming{Colors.ENDC}\n")
            
            try:
                # Use tail -f to stream logs
                import subprocess
                process = subprocess.Popen(
                    ['tail', '-f', str(log_file)],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True
                )
                
                for line in process.stdout:
                    print(line, end='')
                    
            except KeyboardInterrupt:
                print(f"\n{Colors.WARNING}Stopped streaming{Colors.ENDC}")
                process.terminate()
            except Exception as e:
                print(f"{Colors.FAIL}Error streaming logs: {e}{Colors.ENDC}")
        else:
            print(f"{Colors.FAIL}Component not found{Colors.ENDC}")
    
    def _menu_monitor_ping(self):
        """Monitor ping to Jetson"""
        if self.execution_mode != "remote":
            print(f"{Colors.WARNING}Ping monitoring only available in remote mode{Colors.ENDC}")
            return
        
        print(f"\n{Colors.HEADER}Monitoring Jetson Connectivity{Colors.ENDC}")
        print("Press Ctrl+C to stop\n")
        
        try:
            while True:
                if self.ssh_manager.ping():
                    print(f"{Colors.OKGREEN}✓{Colors.ENDC} {datetime.now().strftime('%H:%M:%S')} - Jetson reachable")
                else:
                    print(f"{Colors.FAIL}✗{Colors.ENDC} {datetime.now().strftime('%H:%M:%S')} - Jetson unreachable")
                time.sleep(self.config['network']['ping_interval'])
        except KeyboardInterrupt:
            print("\nStopped monitoring")
    
    def _menu_save_preset(self):
        """Menu for saving a preset"""
        name = input(f"\n{Colors.OKCYAN}Enter preset name: {Colors.ENDC}").strip()
        desc = input(f"{Colors.OKCYAN}Enter description: {Colors.ENDC}").strip()
        
        if name:
            self.state_manager.save_preset(
                name=name,
                process_manager=self.process_manager,
                mode=self.execution_mode,
                active_group=self.current_group,
                description=desc
            )
            print(f"{Colors.OKGREEN}✓ Preset '{name}' saved{Colors.ENDC}")
        else:
            print(f"{Colors.FAIL}Invalid name{Colors.ENDC}")
    
    def _menu_load_preset(self):
        """Menu for loading a preset"""
        presets = self.state_manager.list_presets()
        
        if not presets:
            print(f"{Colors.WARNING}No presets available{Colors.ENDC}")
            return
        
        print(f"\n{Colors.BOLD}Available Presets:{Colors.ENDC}")
        preset_names = list(presets.keys())
        for i, name in enumerate(preset_names, 1):
            print(f"  {i}. {name}")
            print(f"      {presets[name]}")
        
        choice = input(f"\n{Colors.OKCYAN}Enter preset number or name: {Colors.ENDC}").strip()
        
        preset_name = None
        if choice.isdigit():
            idx = int(choice) - 1
            if 0 <= idx < len(preset_names):
                preset_name = preset_names[idx]
        elif choice in preset_names:
            preset_name = choice
        
        if preset_name:
            state = self.state_manager.load_preset(preset_name)
            if state:
                self.state_manager.restore_state(self.process_manager, state)
                self.current_group = state.active_group
                print(f"{Colors.OKGREEN}✓ Preset '{preset_name}' loaded{Colors.ENDC}")
            else:
                print(f"{Colors.FAIL}Failed to load preset{Colors.ENDC}")
        else:
            print(f"{Colors.FAIL}Preset not found{Colors.ENDC}")
    
    def _menu_rollback(self):
        """Menu for rolling back to previous state"""
        history = self.state_manager.list_history(limit=10)
        
        if len(history) <= 1:
            print(f"{Colors.WARNING}No previous states available{Colors.ENDC}")
            return
        
        print(f"\n{Colors.BOLD}Recent States:{Colors.ENDC}")
        for i, state_file in enumerate(history[1:], 1):  # Skip current state
            timestamp = state_file.stem.replace('state_', '')
            print(f"  {i}. {timestamp}")
        
        choice = input(f"\n{Colors.OKCYAN}Enter number of states to rollback (default 1): {Colors.ENDC}").strip()
        steps = int(choice) if choice.isdigit() else 1
        
        if self.state_manager.rollback(self.process_manager, steps=steps):
            print(f"{Colors.OKGREEN}✓ Rolled back {steps} step(s){Colors.ENDC}")
        else:
            print(f"{Colors.FAIL}Rollback failed{Colors.ENDC}")
    
    def cleanup(self):
        """Cleanup resources and save state"""
        logging.info("Cleaning up...")
        
        # Save final state
        self.state_manager.save_state(
            self.process_manager,
            mode=self.execution_mode,
            active_group=self.current_group,
            metadata={"shutdown": True}
        )
        
        # Ask to stop components
        status = self.process_manager.get_status()
        running = [name for name, s in status.items() if s['is_running']]
        
        if running:
            response = input(f"\n{Colors.WARNING}Stop all running components? (y/N): {Colors.ENDC}").strip().lower()
            if response == 'y':
                self.process_manager.stop_all()
        
        # Ask to close monitoring session if auto-monitor enabled
        if self.auto_monitor:
            try:
                # Check if session exists
                result = subprocess.run(
                    ["tmux", "has-session", "-t", self.monitor_session],
                    capture_output=True
                )
                
                if result.returncode == 0:
                    response = input(f"\n{Colors.WARNING}Close monitoring session? (y/N): {Colors.ENDC}").strip().lower()
                    if response == 'y':
                        subprocess.run(["tmux", "kill-session", "-t", self.monitor_session])
                        print(f"{Colors.OKGREEN}✓ Monitoring session closed{Colors.ENDC}")
                    else:
                        print(f"\n{Colors.OKCYAN}Monitoring session still running. To view:{Colors.ENDC}")
                        print(f"  tmux attach -t {self.monitor_session}")
                        print(f"\n{Colors.OKCYAN}To close later:{Colors.ENDC}")
                        print(f"  tmux kill-session -t {self.monitor_session}")
            except Exception as e:
                logging.debug(f"Error during monitoring session cleanup: {e}")


def main():
    parser = argparse.ArgumentParser(
        description="Yuvaan Robot System Manager",
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    
    parser.add_argument(
        '--config', '-c',
        type=str,
        default=None,
        help='Path to configuration file (default: auto-detect)'
    )
    
    parser.add_argument(
        '--mode', '-m',
        choices=['remote', 'local'],
        default='remote',
        help='Execution mode: remote (Jetson) or local (laptop)'
    )
    
    parser.add_argument(
        '--setup-ssh',
        action='store_true',
        help='Setup SSH keys and exit'
    )
    
    parser.add_argument(
        '--status',
        action='store_true',
        help='Show status and exit'
    )
    
    parser.add_argument(
        '--start-group',
        type=str,
        help='Start a specific group and exit'
    )
    
    parser.add_argument(
        '--stop-all',
        action='store_true',
        help='Stop all components and exit'
    )
    
    parser.add_argument(
        '--auto-monitor',
        action='store_true',
        help='Automatically open monitoring panes when components start'
    )
    
    args = parser.parse_args()
    
    # Find config file
    if args.config:
        config_file = Path(args.config)
    else:
        # Auto-detect config file
        ws_path = Path.home() / 'yuvaan_ws'
        config_file = ws_path / 'config' / 'yuvaan_config.yaml'
        
        if not config_file.exists():
            print(f"{Colors.FAIL}Configuration file not found: {config_file}{Colors.ENDC}")
            print("Please specify config file with --config option")
            sys.exit(1)
    
    # Create manager
    manager = YuvaanSystemManager(
        config_file=config_file,
        execution_mode=args.mode,
        auto_monitor=args.auto_monitor
    )
    
    try:
        # Handle specific actions
        if args.setup_ssh:
            manager.setup_ssh()
            return
        
        if args.status:
            manager.show_status()
            return
        
        if args.stop_all:
            manager.process_manager.stop_all()
            return
        
        if args.start_group:
            if not manager.setup_ssh():
                sys.exit(1)
            manager.start_group(args.start_group)
            return
        
        # Default: interactive mode
        print(f"\n{Colors.HEADER}{'='*70}")
        print("  Welcome to Yuvaan Robot System Manager")
        print(f"{'='*70}{Colors.ENDC}\n")
        
        # Setup SSH if in remote mode
        if args.mode == "remote":
            if not manager.setup_ssh():
                print(f"\n{Colors.FAIL}SSH setup failed. Please fix connectivity issues.{Colors.ENDC}")
                sys.exit(1)
        
        # Show monitoring session info if enabled
        if args.auto_monitor:
            print(f"\n{Colors.OKCYAN}{'='*70}")
            print("  Auto-Monitoring Enabled")
            print(f"{'='*70}{Colors.ENDC}")
            print(f"\n{Colors.BOLD}Monitoring session created:{Colors.ENDC} {manager.monitor_session}")
            print(f"\nAs you start components, a monitoring pane will open for each one.")
            print(f"To view the monitoring session, run:")
            print(f"  {Colors.OKGREEN}tmux attach -t {manager.monitor_session}{Colors.ENDC}")
            print(f"\nNavigate panes with: Ctrl+B then Arrow Keys")
            print(f"Detach with: Ctrl+B then D\n")
        
        # Start interactive menu
        manager.interactive_menu()
        
    except KeyboardInterrupt:
        print(f"\n\n{Colors.WARNING}Interrupted by user{Colors.ENDC}")
    finally:
        manager.cleanup()


if __name__ == "__main__":
    main()
