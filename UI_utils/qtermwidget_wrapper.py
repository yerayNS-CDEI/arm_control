#!/usr/bin/env python3
"""
Python wrapper for QTermWidget.
Uses PyQt5 to load the native QTermWidget C++ library.
"""

from PyQt5.QtWidgets import QWidget
from PyQt5.QtCore import Qt, QSize
import sip
from ctypes import CDLL, cdll, c_void_p, c_char_p, c_int, POINTER


class QTermWidget(QWidget):
    """
    Python wrapper for the C++ QTermWidget class from libqtermwidget5.
    
    This creates an actual embedded terminal emulator in a Qt widget.
    """
    
    def __init__(self, startNow=1, parent=None):
        """
        Initialize QTermWidget.
        
        Args:
            startNow: If 1 (default), automatically starts the shell
            parent: Parent QWidget
        """
        # Try to load the library and create the actual QTermWidget
        try:
            # Load the QTermWidget library
            self._lib = cdll.LoadLibrary('libqtermwidget5.so.0')
            
            # This is a hack - we create a regular QWidget and try to
            # promote it to a QTermWidget through the Qt plugin system
            super().__init__(parent)
            
            # Set terminal-like styling
            self.setStyleSheet("""
                QTermWidget {
                    background-color: #000000;
                    color: #00ff00;
                }
            """)
            
            # Try to instantiate the actual C++ QTermWidget
            # This requires SIP to create the wrapper
            # Since direct instantiation is complex, we'll fall back to subprocess approach
            self._use_fallback = True
            
            if self._use_fallback:
                self._init_fallback()
            
        except Exception as e:
            print(f"Failed to load QTermWidget library: {e}")
            print("Falling back to subprocess-based terminal")
            super().__init__(parent)
            self._use_fallback = True
            self._init_fallback()
    
    def _init_fallback(self):
        """Initialize fallback terminal using xterm embedded in widget."""
        from PyQt5.QtWidgets import QVBoxLayout
        from PyQt5.QtCore import QProcess
        import subprocess
        
        # Create layout
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        # Embed xterm using QWindow
        # This requires getting the window ID and embedding xterm into it
        self._xterm_process = None
        
        # For now, show a message that the terminal will be embedded
        from PyQt5.QtWidgets import QLabel
        self._placeholder = QLabel("Terminal will appear here", self)
        self._placeholder.setAlignment(Qt.AlignCenter)
        self._placeholder.setStyleSheet("background-color: #000; color: #0f0; font-family: monospace;")
        layout.addWidget(self._placeholder)
    
    def showEvent(self, event):
        """Override showEvent to embed xterm when widget is shown."""
        super().showEvent(event)
        
        if hasattr(self, '_use_fallback') and self._use_fallback and self._xterm_process is None:
            self._embed_xterm()
    
    def _embed_xterm(self):
        """Embed xterm into this widget using the -into option."""
        # Don't embed if already running
        if hasattr(self, '_xterm_process') and self._xterm_process and self._xterm_process.poll() is None:
            # print(f"[DEBUG] xterm already running, skipping embed")
            return
            
        try:
            # Get the window ID of this widget
            win_id = int(self.winId())
            # print(f"[DEBUG] Embedding xterm into window ID: {win_id}")
            
            # Remove placeholder
            if hasattr(self, '_placeholder'):
                self._placeholder.deleteLater()
                del self._placeholder
            
            # Launch xterm embedded into this widget
            import subprocess
            self._xterm_process = subprocess.Popen([
                'xterm',
                '-into', str(win_id),
                '-bg', '#300a24',
                '-fg', '#ffffff',
                '-fa', 'Monospace',
                '-fs', '10',
                '-sb',           # Enable scrollbar
                '-rightbar',     # Position scrollbar on the right
                '-sl', '10000',  # Scrollback lines (10000 lines)
                '+sb',           # Actually show scrollbar
                '-xrm', 'XTerm.vt100.allowWindowOps: true',  # Allow window operations
                '-xrm', 'XTerm.vt100.selectToClipboard: true',  # Copy selection to clipboard
                '-xrm', 'XTerm.vt100.translations: #override \\n Ctrl Shift <Key>C: copy-selection(CLIPBOARD) \\n Ctrl Shift <Key>V: insert-selection(CLIPBOARD)',
                '-e', '/bin/bash'
            ])
            # print(f"[DEBUG] xterm process started with PID: {self._xterm_process.pid}")
            
        except Exception as e:
            print(f"Failed to embed xterm: {e}")
            import traceback
            traceback.print_exc()
    

    
    def setShellProgram(self, program):
        """Set the shell program (not supported in fallback mode)."""
        pass
    
    def setWorkingDirectory(self, directory):
        """Set working directory (not supported in fallback mode)."""
        pass
    
    def setColorScheme(self, name):
        """Set color scheme (not supported in fallback mode)."""
        pass
    
    def sizeHint(self):
        """Return recommended size."""
        return QSize(600, 400)
    
    def closeEvent(self, event):
        """Clean up when widget is closed."""
        if hasattr(self, '_xterm_process') and self._xterm_process:
            self._xterm_process.terminate()
            self._xterm_process.wait()
        super().closeEvent(event)
