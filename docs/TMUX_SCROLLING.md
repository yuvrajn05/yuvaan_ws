# tmux Scrolling & Navigation Guide

## 🖱️ Mouse Scrolling (NOW ENABLED!)

I've configured your tmux to support mouse scrolling!

**Just use your mouse wheel** - it works now! 🎉

No more `^B` appearing - just scroll naturally with your mouse or trackpad.

## ⌨️ Keyboard Scrolling

If you prefer keyboard:

1. **Enter scroll mode:**
   - Press `Ctrl+B` (release both)
   - Then press `[` (left square bracket)
   - You'll see a line number in the corner

2. **Scroll:**
   - Arrow keys: ↑ ↓
   - Page Up/Down
   - Space/Ctrl+F: Page down
   - Ctrl+B: Page up

3. **Exit scroll mode:**
   - Press `q`

## 🔍 Navigation Shortcuts

### Pane Navigation (NO PREFIX NEEDED!)
- `Alt+Arrow Keys` - Move between panes
  - `Alt+Left` - Left pane
  - `Alt+Right` - Right pane
  - `Alt+Up` - Upper pane
  - `Alt+Down` - Lower pane

### With Ctrl+B Prefix
- `Ctrl+B` then `Arrow Keys` - Navigate panes
- `Ctrl+B` then `Z` - Zoom current pane (toggle fullscreen)
- `Ctrl+B` then `D` - Detach from session
- `Ctrl+B` then `[` - Scroll mode

## 📋 Other Useful Commands

### Splitting (with prefix)
- `Ctrl+B` then `|` - Split horizontally
- `Ctrl+B` then `-` - Split vertically

### Session Management
```bash
# List sessions
tmux list-sessions

# Attach to session
tmux attach -t yuvaan_auto_monitor

# Kill session
tmux kill-session -t yuvaan_auto_monitor

# Reload config
Ctrl+B then r
```

## 🎯 Quick Test

1. Attach to your monitor:
   ```bash
   tmux attach -t yuvaan_auto_monitor
   ```

2. **Use your mouse wheel to scroll** - it should work!

3. Or press `Ctrl+B` then `[` for keyboard scrolling

4. Press `q` to exit scroll mode

---

**The key change:** Mouse support is now enabled in `~/.tmux.conf`!

Just scroll with your mouse wheel normally. No more `^B` characters! 🚀
