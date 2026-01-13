#!/usr/bin/env python3
"""
GUI control panel for Franka robot arm recording and replay.
Provides one-click buttons to start/stop recording and replay.
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import threading
import glob
import os


class FrankaControlGUI:
    def __init__(self):
        # Initialize ROS2
        rclpy.init()
        self.node = Node('franka_control_gui')

        # Service clients
        self.record_client = self.node.create_client(SetBool, 'franka_recorder/start_stop')
        self.replay_client = self.node.create_client(SetBool, 'franka_replay/start_stop')
        self.list_client = self.node.create_client(Trigger, 'franka_replay/list_recordings')
        self.param_client = self.node.create_client(SetParameters, '/franka_replay_node/set_parameters')
        self.check_client = self.node.create_client(Trigger, 'franka_replay/check_position')

        # State variables
        self.is_recording = False
        self.is_replaying = False
        self.data_dir = os.path.expanduser('~/gello_software/franka_recordings')
        os.makedirs(self.data_dir, exist_ok=True)

        # Create GUI
        self.root = tk.Tk()
        self.root.title("Franka Robot Recorder & Replay")
        self.root.geometry("600x750")
        self.root.resizable(True, True)

        # Start ROS2 spinning in background thread
        self.ros_thread = threading.Thread(target=self.spin_ros, daemon=True)
        self.ros_thread.start()

        self.setup_gui()

    def setup_gui(self):
        """Setup the GUI layout"""
        # Configure root grid
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        # Main frame with padding
        main_frame = ttk.Frame(self.root, padding="20")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        main_frame.columnconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)

        # Title
        title_label = ttk.Label(
            main_frame,
            text="Franka Robot Control",
            font=("Arial", 18, "bold")
        )
        title_label.grid(row=0, column=0, columnspan=2, pady=(0, 20))

        # Recording Section
        record_frame = ttk.LabelFrame(main_frame, text="Recording", padding="10")
        record_frame.grid(row=1, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=10)

        self.record_button = tk.Button(
            record_frame,
            text="🔴 Start Recording",
            command=self.toggle_recording,
            bg="#4CAF50",
            fg="white",
            font=("Arial", 14, "bold"),
            height=2,
            width=20,
            relief=tk.RAISED,
            bd=3
        )
        self.record_button.pack(pady=5)

        self.record_status = ttk.Label(
            record_frame,
            text="Status: Ready",
            font=("Arial", 10)
        )
        self.record_status.pack(pady=5)

        # Replay Section
        replay_frame = ttk.LabelFrame(main_frame, text="Replay", padding="10")
        replay_frame.grid(row=2, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=10)

        # Recording selection
        select_frame = ttk.Frame(replay_frame)
        select_frame.pack(fill=tk.X, pady=5)

        ttk.Label(select_frame, text="Select Recording:").pack(side=tk.LEFT, padx=5)

        self.recording_var = tk.StringVar()
        self.recording_combo = ttk.Combobox(
            select_frame,
            textvariable=self.recording_var,
            state="readonly",
            width=35
        )
        self.recording_combo.pack(side=tk.LEFT, padx=5)

        refresh_btn = ttk.Button(
            select_frame,
            text="🔄",
            command=self.refresh_recordings,
            width=3
        )
        refresh_btn.pack(side=tk.LEFT)

        # Replay button
        self.replay_button = tk.Button(
            replay_frame,
            text="▶️  Start Replay",
            command=self.toggle_replay,
            bg="#2196F3",
            fg="white",
            font=("Arial", 14, "bold"),
            height=2,
            width=20,
            relief=tk.RAISED,
            bd=3
        )
        self.replay_button.pack(pady=10)

        self.replay_status = ttk.Label(
            replay_frame,
            text="Status: Ready",
            font=("Arial", 10)
        )
        self.replay_status.pack(pady=5)

        # Info Section
        info_frame = ttk.LabelFrame(main_frame, text="Information", padding="10")
        info_frame.grid(row=3, column=0, columnspan=2, sticky=(tk.W, tk.E, tk.N, tk.S), pady=10)

        info_text = (
            "Recording: Records real Franka robot arm data\n"
            "from /franka/joint_states topic.\n\n"
            "Replay: \n"
            "  ✅ Automatically moves to start position\n"
            "     (same method as FR3 startup)\n"
            "  ✅ Then replays the recorded trajectory\n"
            "  ✅ No manual alignment needed!\n\n"
            f"Recordings saved to:\n{self.data_dir}"
        )

        info_label = ttk.Label(
            info_frame,
            text=info_text,
            font=("Arial", 9),
            justify=tk.LEFT,
            wraplength=550
        )
        info_label.pack(fill=tk.BOTH, expand=True)

        # Open folder button
        open_folder_btn = ttk.Button(
            main_frame,
            text="📁 Open Recordings Folder",
            command=self.open_recordings_folder
        )
        open_folder_btn.grid(row=4, column=0, columnspan=2, pady=10)

        # Initial refresh of recordings
        self.refresh_recordings()

    def spin_ros(self):
        """Spin ROS2 in background thread"""
        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def wait_for_service(self, client, timeout=5.0):
        """Wait for a service to be available"""
        if not client.wait_for_service(timeout_sec=timeout):
            self.node.get_logger().warn(f'Service {client.srv_name} not available')
            return False
        return True

    def toggle_recording(self):
        """Toggle recording on/off"""
        if not self.wait_for_service(self.record_client):
            messagebox.showerror("Error", "Recorder service not available.\nMake sure recorder_node is running.")
            return

        request = SetBool.Request()
        request.data = not self.is_recording

        future = self.record_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)

        if future.result() is not None:
            response = future.result()
            if response.success:
                self.is_recording = not self.is_recording
                if self.is_recording:
                    self.record_button.config(
                        text="⏹️  Stop Recording",
                        bg="#f44336"
                    )
                    self.record_status.config(text="Status: Recording...")
                else:
                    self.record_button.config(
                        text="🔴 Start Recording",
                        bg="#4CAF50"
                    )
                    self.record_status.config(text="Status: Ready")
                    self.refresh_recordings()  # Refresh list after recording

                messagebox.showinfo("Success", response.message)
            else:
                messagebox.showerror("Error", response.message)
        else:
            messagebox.showerror("Error", "Failed to call recording service")

    def toggle_replay(self):
        """Toggle replay on/off"""
        if not self.is_replaying:
            # Load selected recording first
            selected = self.recording_var.get()
            if not selected:
                messagebox.showerror("Error", "Please select a recording first")
                return

            # Set the recording_file parameter
            if not self.wait_for_service(self.param_client):
                messagebox.showerror("Error", "Replay parameter service not available.\nMake sure replay_node is running.")
                return

            # Create parameter to set the recording file
            param = Parameter()
            param.name = 'recording_file'
            param.value = ParameterValue()
            param.value.type = ParameterType.PARAMETER_STRING
            param.value.string_value = selected

            param_request = SetParameters.Request()
            param_request.parameters = [param]

            param_future = self.param_client.call_async(param_request)
            rclpy.spin_until_future_complete(self.node, param_future, timeout_sec=5.0)

            if param_future.result() is None or not param_future.result().results[0].successful:
                messagebox.showerror("Error", "Failed to set recording file")
                return

            self.node.get_logger().info(f'Set recording file to: {selected}')

            # Show info about auto-alignment
            messagebox.showinfo(
                "Auto-Alignment Enabled",
                "The robot will automatically move to the recording start position\n"
                "before replaying the trajectory.\n\n"
                "This takes about 5 seconds.\n\n"
                "Click OK to start replay."
            )

        # Start/stop replay
        if not self.wait_for_service(self.replay_client):
            messagebox.showerror("Error", "Replay service not available")
            return

        request = SetBool.Request()
        request.data = not self.is_replaying

        future = self.replay_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)

        if future.result() is not None:
            response = future.result()
            if response.success:
                self.is_replaying = not self.is_replaying
                if self.is_replaying:
                    self.replay_button.config(
                        text="⏹️  Stop Replay",
                        bg="#f44336"
                    )
                    self.replay_status.config(text="Status: Replaying...")
                else:
                    self.replay_button.config(
                        text="▶️  Start Replay",
                        bg="#2196F3"
                    )
                    self.replay_status.config(text="Status: Ready")

                messagebox.showinfo("Success", response.message)
            else:
                messagebox.showerror("Error", response.message)
        else:
            messagebox.showerror("Error", "Failed to call replay service")

    def refresh_recordings(self):
        """Refresh the list of available recordings"""
        recordings = sorted(glob.glob(os.path.join(self.data_dir, '*.pkl')))
        filenames = [os.path.basename(f) for f in recordings]

        self.recording_combo['values'] = filenames
        if filenames:
            self.recording_combo.current(len(filenames) - 1)  # Select latest

    def open_recordings_folder(self):
        """Open the recordings folder in file manager"""
        import subprocess
        import platform

        if platform.system() == 'Linux':
            subprocess.Popen(['xdg-open', self.data_dir])
        elif platform.system() == 'Darwin':  # macOS
            subprocess.Popen(['open', self.data_dir])
        elif platform.system() == 'Windows':
            subprocess.Popen(['explorer', self.data_dir])

    def run(self):
        """Run the GUI main loop"""
        self.root.mainloop()

    def cleanup(self):
        """Cleanup on exit"""
        self.node.destroy_node()
        rclpy.shutdown()


def main():
    try:
        gui = FrankaControlGUI()
        gui.run()
    except KeyboardInterrupt:
        pass
    finally:
        if 'gui' in locals():
            gui.cleanup()


if __name__ == '__main__':
    main()
