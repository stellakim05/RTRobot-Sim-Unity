import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path

class ExperimentPlotter:
    def __init__(self, csv_file):
        self.data = self.read_data_file(csv_file)
        self.setup_plot_style()
        self.detect_events()

    def read_data_file(self, csv_file):
        """Read CSV file and exclude summary section"""
        try:
            with open(csv_file, 'r') as f:
                lines = []
                for line in f:
                    if line.strip() == '' or 'Session Summary' in line:
                        break
                    lines.append(line)
            
            from io import StringIO
            data = pd.read_csv(StringIO(''.join(lines)))
            
            # Convert numeric columns
            numeric_columns = ['TaskTime', 'BoxRotation', 'ForceMagnitude', 'IsInContact']
            for col in numeric_columns:
                if col in data.columns:
                    data[col] = pd.to_numeric(data[col], errors='coerce')
            
            return data.fillna(0)
        except Exception as e:
            print(f"Error reading file: {e}")
            return pd.DataFrame()

    def setup_plot_style(self):
        """Set up global plot style with larger, more visible elements"""
        plt.style.use('default')
        plt.rcParams['figure.figsize'] = [16, 12]
        plt.rcParams['font.size'] = 14          # Base font size
        plt.rcParams['axes.labelsize'] = 16     # Axis labels
        plt.rcParams['axes.titlesize'] = 18     # Subplot titles
        plt.rcParams['figure.titlesize'] = 20   # Figure title
        plt.rcParams['xtick.labelsize'] = 14    # X-axis tick labels
        plt.rcParams['ytick.labelsize'] = 14    # Y-axis tick labels
        plt.rcParams['legend.fontsize'] = 14    # Legend text
        plt.rcParams['axes.grid'] = True        # Show grid
        plt.rcParams['grid.alpha'] = 0.3        # Grid transparency
        plt.rcParams['lines.linewidth'] = 2.5   # Line thickness
        plt.rcParams['axes.linewidth'] = 2      # Axis line thickness
        plt.rcParams['grid.linewidth'] = 1.5    # Grid line thickness

    def detect_events(self):
        """Detect important events in the experiment"""
        self.events = {}
        
        # Detect first contact
        contact_mask = self.data['IsInContact'] == 1
        if contact_mask.any():
            first_contact = self.data[contact_mask].iloc[0]
            self.events['First Contact'] = first_contact['TaskTime']

        # Detect significant rotation (> 5 degrees)
        rotation_mask = abs(self.data['BoxRotation']) > 5
        if rotation_mask.any():
            sig_rotation = self.data[rotation_mask].iloc[0]
            self.events['Significant Rotation'] = sig_rotation['TaskTime']

        print("\nDetected Events:")
        for event, time in self.events.items():
            print(f"{event}: {time:.2f}s")

    def plot_experiment_timeline(self):
        """Plot main experiment timeline with improved annotations and data clarity"""
        fig = plt.figure(figsize=(16, 12))
        gs = plt.GridSpec(4, 1, height_ratios=[1, 1, 1, 0.3], hspace=0.3)

        # Box Rotation with improved scaling and error bars
        ax1 = fig.add_subplot(gs[0])
        ax1.plot(self.data['TaskTime'], self.data['BoxRotation'], 
                color='blue', linewidth=2.5, label='Rotation')
        # Add error bounds (assuming 1-degree measurement uncertainty)
        ax1.fill_between(self.data['TaskTime'], 
                        self.data['BoxRotation'] - 1, 
                        self.data['BoxRotation'] + 1, 
                        color='blue', alpha=0.2)
        ax1.set_ylabel('Box Rotation (°)', fontsize=16)
        ax1.grid(True, alpha=0.3)
        ax1.legend(loc='upper right', fontsize=14, framealpha=0.9)

        # Robot Speeds with filtered data
        ax2 = fig.add_subplot(gs[1])
        # Apply moving average filter to smooth speed data
        window = 5  # Adjust window size as needed
        robot1_speed_smooth = self.data['Robot1Speed'].rolling(window=window, center=True).mean()
        robot2_speed_smooth = self.data['Robot2Speed'].rolling(window=window, center=True).mean()
        
        ax2.plot(self.data['TaskTime'], robot1_speed_smooth, 
                color='red', linewidth=2.5, label='Robot 1')
        ax2.plot(self.data['TaskTime'], robot2_speed_smooth, 
                color='green', linewidth=2.5, label='Robot 2')
        ax2.set_ylabel('Speed (m/s)', fontsize=16)
        ax2.grid(True, alpha=0.3)
        ax2.legend(loc='upper right', fontsize=14, framealpha=0.9)

        # Force and Contact with improved visualization
        ax3 = fig.add_subplot(gs[2])
        contact_mask = self.data['IsInContact'] == 1
        ax3.plot(self.data['TaskTime'], self.data['ForceMagnitude'], 
                color='purple', linewidth=2.5, label='Force')
        
        # Separate contact types and intensities
        box_contacts = contact_mask & (self.data['ContactCount'] == 1)
        robot_contacts = contact_mask & (self.data['ContactCount'] > 1)
        
        if box_contacts.any():
            ax3.scatter(self.data['TaskTime'][box_contacts], 
                    self.data['ForceMagnitude'][box_contacts],
                    color='blue', s=100, alpha=0.5, label='Box Contact')
        if robot_contacts.any():
            ax3.scatter(self.data['TaskTime'][robot_contacts], 
                    self.data['ForceMagnitude'][robot_contacts],
                    color='red', s=100, alpha=0.5, label='Robot Contact')
        
        ax3.set_ylabel('Force (N)', fontsize=16)
        ax3.grid(True, alpha=0.3)
        ax3.legend(loc='upper right', fontsize=14, framealpha=0.9)

        # Add interaction phase timeline
        ax4 = fig.add_subplot(gs[3])
        self.plot_interaction_phases(ax4)

        # Add event markers with improved positioning and descriptions
        self.add_event_markers([ax1, ax2, ax3])

        # Add coordinate system annotation
        coord_text = "Force Coordinate System:\nX: Left/Right\nY: Up/Down\nZ: Forward/Back"
        fig.text(0.02, 0.02, coord_text, fontsize=12, 
                bbox=dict(facecolor='white', alpha=0.8))

        fig.suptitle('Experiment Timeline', fontsize=20, y=0.95)
        return fig
    
    def plot_interaction_phases(self, ax):
        """Plot interaction phase timeline"""
        phases = []
        times = []
        current_phase = "No Contact"
        
        for idx, row in self.data.iterrows():
            if row['IsInContact'] == 0:
                phase = "No Contact"
            elif row['ContactCount'] == 1:
                phase = "Box Contact"
            else:
                phase = "Robot Contact"
                
            if phase != current_phase:
                phases.append(phase)
                times.append(row['TaskTime'])
                current_phase = phase

        # Plot phases as colored regions
        colors = {'No Contact': 'white', 'Box Contact': 'lightblue', 'Robot Contact': 'lightcoral'}
        
        for i in range(len(times)-1):
            ax.axvspan(times[i], times[i+1], 
                    color=colors[phases[i]], 
                    alpha=0.5, 
                    label=phases[i] if phases[i] not in ax.get_legend_handles_labels()[1] else "")

        ax.set_yticks([])
        ax.set_xlabel('Time (s)', fontsize=16)
        ax.legend(loc='center right', bbox_to_anchor=(1.15, 0.5))

    def add_event_markers(self, axes):
        """Add event markers with improved descriptions"""
        event_descriptions = {
            'First Contact': 'Initial box contact',
            'Robot Contact': 'Transition to robot interaction',
            'Max Force': 'Peak force application',
            'Movement Start': 'Robots begin moving',
            'Movement Stop': 'Robots stop moving'
        }

        # Improved movement detection
        speed_threshold = 0.05  # Increased threshold to filter out noise (was too low)
        window_size = 5  # For smoothing
        
        # Smooth the speed data to reduce noise
        robot1_speed_smooth = self.data['Robot1Speed'].rolling(window=window_size, center=True).mean()
        robot2_speed_smooth = self.data['Robot2Speed'].rolling(window=window_size, center=True).mean()
        
        # Combined movement mask with higher threshold
        movement_mask = (robot1_speed_smooth > speed_threshold) | \
                    (robot2_speed_smooth > speed_threshold)
        
        # Find transitions (0 to 1 for start, 1 to 0 for stop)
        movement_transitions = np.diff(movement_mask.astype(int))
        
        # Get timestamps of transitions
        movement_starts = self.data['TaskTime'][1:][movement_transitions > 0]
        movement_stops = self.data['TaskTime'][1:][movement_transitions < 0]

        # Only add movement events if actual movement detected
        if len(movement_starts) > 0:
            self.events['Movement Start'] = movement_starts.iloc[0]
        if len(movement_stops) > 0:
            self.events['Movement Stop'] = movement_stops.iloc[-1]  # Use last stop if multiple

        # Add event markers with staggered heights and detailed descriptions
        for i, (event, time) in enumerate(self.events.items()):
            for ax in axes:
                ymin, ymax = ax.get_ylim()
                text_y = ymax - (i + 1) * (ymax - ymin) * 0.15
                
                ax.axvline(x=time, color='black', linestyle='--', 
                        alpha=0.5, linewidth=2)
                ax.text(time + 0.5, text_y, 
                    f"{event}\n{event_descriptions.get(event, '')}",
                    fontsize=12,
                    bbox=dict(facecolor='white',
                                edgecolor='black',
                                alpha=0.8,
                                pad=5))

        # Add debug log for movement detection
        if self.data['Robot1Speed'].max() < speed_threshold and \
        self.data['Robot2Speed'].max() < speed_threshold:
            print(f"No significant movement detected. Max speeds: " \
                f"Robot1={self.data['Robot1Speed'].max():.3f}, " \
                f"Robot2={self.data['Robot2Speed'].max():.3f}")

    def plot_robot_interaction(self):
            """Plot robot positions and distances with improved visibility"""
            fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(16, 12))
            plt.subplots_adjust(hspace=0.3)

            # Robot positions
            ax1.plot(self.data['TaskTime'], self.data['Robot1PosX'], 
                    color='red', linewidth=2.5, label='Robot 1')
            ax1.plot(self.data['TaskTime'], self.data['Robot2PosX'], 
                    color='blue', linewidth=2.5, label='Robot 2')
            ax1.set_ylabel('X Position (m)', fontsize=16)
            ax1.grid(True, alpha=0.3)
            ax1.legend(loc='center right', fontsize=14, 
                    bbox_to_anchor=(1.15, 0.5),
                    framealpha=0.9)
            ax1.set_title('Robot Positions', pad=20, fontsize=18)

            # Robot distance
            distance = self.data['RobotDistanceDiff']
            ax2.plot(self.data['TaskTime'], distance, 
                    color='green', linewidth=2.5)
            ax2.set_xlabel('Time (s)', fontsize=16)
            ax2.set_ylabel('Distance (m)', fontsize=16)
            ax2.grid(True, alpha=0.3)
            ax2.set_title('Distance Between Robots', pad=20, fontsize=18)

            # Add event markers with improved positioning
            for i, (event, time) in enumerate(self.events.items()):
                for ax in [ax1, ax2]:
                    ax.axvline(x=time, color='black', linestyle='--', 
                            alpha=0.5, linewidth=2)
                    ymin, ymax = ax.get_ylim()
                    # Stagger text positions
                    text_y = ymax - (i + 1) * (ymax - ymin) * 0.15
                    ax.text(time + 0.5, text_y, event,
                        fontsize=14,
                        bbox=dict(facecolor='white',
                                    edgecolor='black',
                                    alpha=0.8,
                                    pad=5))

            fig.suptitle('Robot Interaction Analysis', fontsize=20, y=0.95)
            return fig

    def plot_haptic_analysis(self):
        """Plot haptic interaction analysis with improved visibility"""
        fig = plt.figure(figsize=(16, 12))
        gs = plt.GridSpec(2, 2, figure=fig)
        ax1 = fig.add_subplot(gs[0, 0])
        ax2 = fig.add_subplot(gs[0, 1])
        ax3 = fig.add_subplot(gs[1, 0])
        ax4 = fig.add_subplot(gs[1, 1])

        try:
            # Force components during contact
            contact_mask = self.data['IsInContact'] == 1
            if contact_mask.any():
                contact_data = self.data[contact_mask]
                ax1.plot(contact_data['TaskTime'], contact_data['HapticForceX'], 
                        color='red', linewidth=2.5, label='X')
                ax1.plot(contact_data['TaskTime'], contact_data['HapticForceY'], 
                        color='green', linewidth=2.5, label='Y')
                ax1.plot(contact_data['TaskTime'], contact_data['HapticForceZ'], 
                        color='blue', linewidth=2.5, label='Z')
            ax1.set_ylabel('Force (N)', fontsize=16)
            ax1.grid(True, alpha=0.3)
            ax1.legend(loc='upper right', fontsize=14, framealpha=0.9)
            ax1.set_title('Force Components During Contact', pad=20, fontsize=18)

            # Force distribution histogram
            force_data = self.data[contact_mask]['ForceMagnitude']
            if not force_data.empty:
                bins = np.linspace(0, force_data.max(), 30)
                ax2.hist(force_data, bins=bins, color='blue', alpha=0.7,
                        edgecolor='black', linewidth=1.5)
                ax2.set_xlabel('Force Magnitude (N)', fontsize=16)
                ax2.set_ylabel('Count', fontsize=16)
                ax2.grid(True, alpha=0.3)
                ax2.set_title('Force Distribution', pad=20, fontsize=18)

            # Force vs Rotation with time-based coloring
            if contact_mask.any():
                times = pd.to_numeric(contact_data['TaskTime'])
                scatter = ax3.scatter(contact_data['ForceMagnitude'],
                                   contact_data['BoxRotation'],
                                   c=times,
                                   cmap='viridis',
                                   s=100,  # Increased marker size
                                   alpha=0.6)
                cbar = plt.colorbar(scatter, ax=ax3)
                cbar.set_label('Time (s)', fontsize=14, labelpad=15)
                cbar.ax.tick_params(labelsize=12)
            ax3.set_xlabel('Force Magnitude (N)', fontsize=16)
            ax3.set_ylabel('Box Rotation (°)', fontsize=16)
            ax3.grid(True, alpha=0.3)
            ax3.set_title('Force-Rotation Relationship', pad=20, fontsize=18)

            # Cumulative contact duration
            times = pd.to_numeric(self.data['TaskTime'])
            cumulative_contact = np.cumsum(self.data['IsInContact']) * \
                               np.mean(np.diff(times))
            ax4.plot(times, cumulative_contact, 
                    color='black', linewidth=2.5)
            ax4.set_xlabel('Time (s)', fontsize=16)
            ax4.set_ylabel('Contact Duration (s)', fontsize=16)
            ax4.grid(True, alpha=0.3)
            ax4.set_title('Cumulative Contact Time', pad=20, fontsize=18)

        except Exception as e:
            print(f"Error in haptic analysis plot: {e}")

        plt.tight_layout()
        fig.suptitle('Haptic Interaction Analysis', fontsize=20, y=1.02)
        return fig

    def save_all_plots(self, output_dir):
        """Save all plots to files with high resolution"""
        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)
        
        plots = {
            'timeline': self.plot_experiment_timeline(),
            'robot_interaction': self.plot_robot_interaction(),
            'haptic_analysis': self.plot_haptic_analysis()
        }
        
        for name, fig in plots.items():
            filepath = output_path / f'{name}.png'
            fig.savefig(filepath, 
                       dpi=300,
                       bbox_inches='tight',
                       pad_inches=0.5)
            plt.close(fig)
            print(f'Saved {filepath}')

if __name__ == "__main__":
    try:
        plotter = ExperimentPlotter("Assets\ExperimentData\experiment_session_20241106_021132.csv")
        plotter.save_all_plots("experiment_plots")
        print("All plots generated successfully!")
    except Exception as e:
        print(f"Error occurred: {str(e)}")
        import traceback
        traceback.print_exc()