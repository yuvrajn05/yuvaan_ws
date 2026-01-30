#!/usr/bin/env python3
"""
State Manager for Yuvaan Robot System
Handles system state persistence, rollback, and presets
"""

import json
import logging
from pathlib import Path
from datetime import datetime
from typing import Dict, List, Optional
from dataclasses import dataclass, asdict
from enum import Enum

logger = logging.getLogger(__name__)


@dataclass
class ComponentState:
    """State of a single component"""
    name: str
    running: bool
    restart_count: int
    uptime: Optional[float]
    timestamp: str


@dataclass
class SystemState:
    """Complete system state"""
    timestamp: str
    mode: str  # remote/local
    active_group: Optional[str]
    components: List[ComponentState]
    metadata: Dict


class StateManager:
    """Manages system state persistence and rollback"""
    
    def __init__(self, state_dir: Path):
        self.state_dir = Path(state_dir).expanduser()
        self.state_dir.mkdir(parents=True, exist_ok=True)
        
        self.current_state_file = self.state_dir / "current_state.json"
        self.history_dir = self.state_dir / "history"
        self.history_dir.mkdir(exist_ok=True)
        
        self.presets_file = self.state_dir / "presets.json"
        self.presets: Dict[str, SystemState] = {}
        
        self._load_presets()
    
    def save_state(self, process_manager, mode: str, active_group: Optional[str] = None,
                  metadata: Optional[Dict] = None) -> bool:
        """Save current system state"""
        try:
            # Collect component states
            component_states = []
            status = process_manager.get_status()
            
            for name, proc_status in status.items():
                comp_state = ComponentState(
                    name=name,
                    running=proc_status.get('is_running', False),
                    restart_count=proc_status.get('restart_count', 0),
                    uptime=proc_status.get('uptime'),
                    timestamp=datetime.now().isoformat()
                )
                component_states.append(comp_state)
            
            # Create system state
            state = SystemState(
                timestamp=datetime.now().isoformat(),
                mode=mode,
                active_group=active_group,
                components=component_states,
                metadata=metadata or {}
            )
            
            # Save to current state file
            with open(self.current_state_file, 'w') as f:
                json.dump(asdict(state), f, indent=2)
            
            # Save to history
            history_file = self.history_dir / f"state_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
            with open(history_file, 'w') as f:
                json.dump(asdict(state), f, indent=2)
            
            logger.info(f"State saved to {self.current_state_file}")
            
            # Cleanup old history files (keep last 20)
            self._cleanup_history(keep=20)
            
            return True
            
        except Exception as e:
            logger.error(f"Error saving state: {e}")
            return False
    
    def load_state(self, state_file: Optional[Path] = None) -> Optional[SystemState]:
        """Load system state from file"""
        try:
            file_to_load = state_file or self.current_state_file
            
            if not file_to_load.exists():
                logger.warning(f"State file not found: {file_to_load}")
                return None
            
            with open(file_to_load, 'r') as f:
                data = json.load(f)
            
            # Reconstruct SystemState
            components = [ComponentState(**c) for c in data['components']]
            state = SystemState(
                timestamp=data['timestamp'],
                mode=data['mode'],
                active_group=data.get('active_group'),
                components=components,
                metadata=data.get('metadata', {})
            )
            
            return state
            
        except Exception as e:
            logger.error(f"Error loading state: {e}")
            return None
    
    def restore_state(self, process_manager, state: SystemState) -> bool:
        """Restore system to a previous state"""
        try:
            logger.info(f"Restoring state from {state.timestamp}")
            
            # Get current status
            current_status = process_manager.get_status()
            
            # Determine what needs to change
            to_start = []
            to_stop = []
            
            state_dict = {c.name: c for c in state.components}
            
            for name in current_status.keys():
                current_running = current_status[name]['is_running']
                should_run = state_dict.get(name, ComponentState(name, False, 0, None, "")).running
                
                if should_run and not current_running:
                    to_start.append(name)
                elif not should_run and current_running:
                    to_stop.append(name)
            
            # Stop processes first
            for name in to_stop:
                logger.info(f"Stopping {name} (not in target state)")
                process_manager.stop_process(name)
            
            # Start processes
            for name in to_start:
                logger.info(f"Starting {name} (required in target state)")
                process_manager.start_process(name)
            
            logger.info("State restored successfully")
            return True
            
        except Exception as e:
            logger.error(f"Error restoring state: {e}")
            return False
    
    def save_preset(self, name: str, process_manager, mode: str, 
                   active_group: Optional[str] = None, description: str = "") -> bool:
        """Save current state as a named preset"""
        try:
            # Collect component states
            component_states = []
            status = process_manager.get_status()
            
            for proc_name, proc_status in status.items():
                if proc_status.get('is_running', False):
                    comp_state = ComponentState(
                        name=proc_name,
                        running=True,
                        restart_count=0,
                        uptime=None,
                        timestamp=datetime.now().isoformat()
                    )
                    component_states.append(comp_state)
            
            # Create preset state
            preset = SystemState(
                timestamp=datetime.now().isoformat(),
                mode=mode,
                active_group=active_group,
                components=component_states,
                metadata={"description": description}
            )
            
            self.presets[name] = preset
            self._save_presets()
            
            logger.info(f"Preset '{name}' saved")
            return True
            
        except Exception as e:
            logger.error(f"Error saving preset: {e}")
            return False
    
    def load_preset(self, name: str) -> Optional[SystemState]:
        """Load a named preset"""
        return self.presets.get(name)
    
    def list_presets(self) -> Dict[str, str]:
        """List all available presets with descriptions"""
        return {
            name: state.metadata.get('description', 'No description')
            for name, state in self.presets.items()
        }
    
    def list_history(self, limit: int = 10) -> List[Path]:
        """List recent state history files"""
        history_files = sorted(
            self.history_dir.glob("state_*.json"),
            key=lambda p: p.stat().st_mtime,
            reverse=True
        )
        return history_files[:limit]
    
    def get_state_diff(self, state1: SystemState, state2: SystemState) -> Dict:
        """Compare two states and return differences"""
        diff = {
            'mode_changed': state1.mode != state2.mode,
            'group_changed': state1.active_group != state2.active_group,
            'components_started': [],
            'components_stopped': [],
        }
        
        state1_running = {c.name for c in state1.components if c.running}
        state2_running = {c.name for c in state2.components if c.running}
        
        diff['components_started'] = list(state2_running - state1_running)
        diff['components_stopped'] = list(state1_running - state2_running)
        
        return diff
    
    def rollback(self, process_manager, steps: int = 1) -> bool:
        """Rollback to a previous state from history"""
        try:
            history = self.list_history(limit=steps + 1)
            
            if len(history) <= steps:
                logger.error(f"Not enough history to rollback {steps} steps")
                return False
            
            # Load the target state
            target_state = self.load_state(history[steps])
            
            if not target_state:
                logger.error("Failed to load rollback state")
                return False
            
            logger.info(f"Rolling back to state from {target_state.timestamp}")
            return self.restore_state(process_manager, target_state)
            
        except Exception as e:
            logger.error(f"Error during rollback: {e}")
            return False
    
    def _save_presets(self):
        """Save presets to file"""
        try:
            presets_data = {
                name: asdict(state)
                for name, state in self.presets.items()
            }
            
            with open(self.presets_file, 'w') as f:
                json.dump(presets_data, f, indent=2)
                
        except Exception as e:
            logger.error(f"Error saving presets: {e}")
    
    def _load_presets(self):
        """Load presets from file"""
        try:
            if not self.presets_file.exists():
                return
            
            with open(self.presets_file, 'r') as f:
                presets_data = json.load(f)
            
            for name, data in presets_data.items():
                components = [ComponentState(**c) for c in data['components']]
                state = SystemState(
                    timestamp=data['timestamp'],
                    mode=data['mode'],
                    active_group=data.get('active_group'),
                    components=components,
                    metadata=data.get('metadata', {})
                )
                self.presets[name] = state
                
        except Exception as e:
            logger.error(f"Error loading presets: {e}")
    
    def _cleanup_history(self, keep: int = 20):
        """Remove old history files, keeping only the most recent"""
        try:
            history_files = sorted(
                self.history_dir.glob("state_*.json"),
                key=lambda p: p.stat().st_mtime,
                reverse=True
            )
            
            # Remove old files
            for old_file in history_files[keep:]:
                old_file.unlink()
                logger.debug(f"Removed old history file: {old_file}")
                
        except Exception as e:
            logger.error(f"Error cleaning up history: {e}")


if __name__ == "__main__":
    # Test state manager
    logging.basicConfig(level=logging.INFO)
    
    sm = StateManager(Path("~/test_state"))
    
    # Test preset listing
    print("Presets:", sm.list_presets())
    print("History:", sm.list_history())
