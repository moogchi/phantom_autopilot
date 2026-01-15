import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import serial
import serial.tools.list_ports

def quaternion_to_rotation_matrix(q):
    """
    Convert quaternion to rotation matrix.
    q = [w, x, y, z] format
    """
    # Format is [w, x, y, z] from STM32
    w, x, y, z = q
    
    # Normalize quaternion
    norm = np.sqrt(w*w + x*x + y*y + z*z)
    if norm > 0:
        w, x, y, z = w/norm, x/norm, y/norm, z/norm
    
    # Create rotation matrix
    R = np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)]
    ])
    
    return R

def create_airplane_vertices():
    """Create vertices for an airplane shape."""
    # Airplane pointing along +X axis (forward)
    vertices = np.array([
        # Fuselage (body)
        [1.5, 0, 0],      # 0: Nose
        [0.5, 0.1, 0.1],  # 1: Front top
        [0.5, 0.1, -0.1], # 2: Front bottom
        [0.5, -0.1, 0.1], # 3: Front right
        [0.5, -0.1, -0.1],# 4: Front left
        [-0.8, 0.1, 0.1], # 5: Rear top
        [-0.8, 0.1, -0.1],# 6: Rear bottom
        [-0.8, -0.1, 0.1],# 7: Rear right
        [-0.8, -0.1, -0.1],# 8: Rear left
        
        # Wings (extending along Y axis)
        [0.2, 1.5, 0],    # 9: Right wing tip
        [0.2, -1.5, 0],   # 10: Left wing tip
        [-0.2, 1.2, 0],   # 11: Right wing back
        [-0.2, -1.2, 0],  # 12: Left wing back
        
        # Tail
        [-1.0, 0, 0],     # 13: Tail end
        [-0.8, 0, 0.4],   # 14: Vertical stabilizer top
        [-0.8, 0.3, 0],   # 15: Horizontal stabilizer right
        [-0.8, -0.3, 0],  # 16: Horizontal stabilizer left
    ])
    
    return vertices

def create_airplane_faces(vertices):
    """Define the faces of the airplane."""
    faces = [
        # Fuselage sides
        [vertices[0], vertices[1], vertices[5], vertices[13]],  # Top
        [vertices[0], vertices[2], vertices[6], vertices[13]],  # Bottom
        [vertices[0], vertices[3], vertices[7], vertices[13]],  # Right side
        [vertices[0], vertices[4], vertices[8], vertices[13]],  # Left side
        
        # Wings (right)
        [vertices[3], vertices[1], vertices[9], vertices[11]],  # Right wing top
        [vertices[4], vertices[2], vertices[9], vertices[11]],  # Right wing bottom
        
        # Wings (left)
        [vertices[1], vertices[3], vertices[10], vertices[12]],  # Left wing top
        [vertices[2], vertices[4], vertices[10], vertices[12]],  # Left wing bottom
        
        # Vertical stabilizer
        [vertices[13], vertices[5], vertices[14]],  # Tail fin
        
        # Horizontal stabilizers
        [vertices[13], vertices[7], vertices[15]],  # Right horizontal
        [vertices[13], vertices[8], vertices[16]],  # Left horizontal
        
        # Nose (red marker)
        [vertices[0], vertices[1], vertices[3]],  # Nose top
        [vertices[0], vertices[2], vertices[4]],  # Nose bottom
    ]
    
    return faces

def list_serial_ports():
    """List all available serial ports."""
    ports = serial.tools.list_ports.comports()
    print("\nAvailable serial ports:")
    for i, port in enumerate(ports):
        print(f"{i}: {port.device} - {port.description}")
    return [port.device for port in ports]

class QuaternionVisualizer:
    def __init__(self, port=None, baudrate=115200):
        # Setup serial connection
        if port is None:
            ports = list_serial_ports()
            if not ports:
                print("No serial ports found!")
                self.ser = None
            else:
                port_idx = input(f"\nSelect port (0-{len(ports)-1}), or press Enter to skip serial: ")
                if port_idx.strip():
                    port = ports[int(port_idx)]
                    try:
                        self.ser = serial.Serial(port, baudrate, timeout=0.1)
                        print(f"Connected to {port} at {baudrate} baud")
                    except Exception as e:
                        print(f"Error opening serial port: {e}")
                        self.ser = None
                else:
                    self.ser = None
                    print("Running in test mode without serial connection")
        else:
            try:
                self.ser = serial.Serial(port, baudrate, timeout=0.1)
                print(f"Connected to {port} at {baudrate} baud")
            except Exception as e:
                print(f"Error opening serial port: {e}")
                self.ser = None
        
        # Current quaternion and sensor data
        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0])  # [w, x, y, z]
        self.accel = np.array([0.0, 0.0, 0.0])  # [ax, ay, az]
        self.gyro = np.array([0.0, 0.0, 0.0])  # [gx, gy, gz]
        self.sim_time = 0.0
        self.dt = 0.0
        
        # Setup plot
        self.fig = plt.figure(figsize=(10, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # Set axis limits
        self.ax.set_xlim([-2, 2])
        self.ax.set_ylim([-2, 2])
        self.ax.set_zlim([-2, 2])
        
        # Set labels
        self.ax.set_xlabel('X (Forward)')
        self.ax.set_ylabel('Y (Right)')
        self.ax.set_zlabel('Z (Up)')
        self.ax.set_title('Flight Orientation Visualization')
        
        # Initialize airplane
        self.airplane_vertices = create_airplane_vertices()
        self.poly = None
        
    def read_sensor_data(self):
        """Read quaternion and sensor data from serial port."""
        if self.ser is None or not self.ser.is_open:
            return None
        
        try:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    # Parse comma-separated values: q[4], accel[3], gyro[3], sim_time, dt
                    values = [float(v.strip()) for v in line.split(',')]
                    if len(values) == 12:
                        # Format: quat[4], accel[3], gyro[3], sim_time, dt
                        return {
                            'quat': np.array(values[0:4]),
                            'accel': np.array(values[4:7]),
                            'gyro': np.array(values[7:10]),
                            'sim_time': values[10],
                            'dt': values[11]
                        }
                    elif len(values) == 10:  # Fallback for old format
                        # Format: quat[4], accel[3], gyro[3]
                        return {
                            'quat': np.array(values[0:4]),
                            'accel': np.array(values[4:7]),
                            'gyro': np.array(values[7:10]),
                            'sim_time': None,
                            'dt': None
                        }
                    elif len(values) == 4:  # Original format
                        return {'quat': np.array(values), 'accel': None, 'gyro': None, 'sim_time': None, 'dt': None}
        except Exception as e:
            print(f"Error reading serial: {e}")
        
        return None
    
    def update(self, frame):
        """Update function for animation."""
        # Read new sensor data
        data = self.read_sensor_data()
        if data is not None and data['quat'] is not None:
            self.quaternion = data['quat']
            if data['accel'] is not None:
                self.accel = data['accel']
                self.gyro = data['gyro']
                if data.get('sim_time') is not None:
                    self.sim_time = data['sim_time']
                    self.dt = data['dt']
                    print(f"Q:[{self.quaternion[0]:6.3f},{self.quaternion[1]:6.3f},{self.quaternion[2]:6.3f},{self.quaternion[3]:6.3f}] "
                          f"A:[{self.accel[0]:6.3f},{self.accel[1]:6.3f},{self.accel[2]:6.3f}] "
                          f"G:[{self.gyro[0]:7.2f},{self.gyro[1]:7.2f},{self.gyro[2]:7.2f}] "
                          f"T:{self.sim_time:7.3f} dt:{self.dt:6.4f}")
                else:
                    print(f"Q:[{self.quaternion[0]:6.3f},{self.quaternion[1]:6.3f},{self.quaternion[2]:6.3f},{self.quaternion[3]:6.3f}] "
                          f"A:[{self.accel[0]:6.3f},{self.accel[1]:6.3f},{self.accel[2]:6.3f}] "
                          f"G:[{self.gyro[0]:7.2f},{self.gyro[1]:7.2f},{self.gyro[2]:7.2f}]")
            else:
                print(f"Quat: [{self.quaternion[0]:.4f}, {self.quaternion[1]:.4f}, {self.quaternion[2]:.4f}, {self.quaternion[3]:.4f}]")
        
        # Apply rotation to airplane
        R = quaternion_to_rotation_matrix(self.quaternion)
        rotated_vertices = np.dot(self.airplane_vertices, R.T)
        
        # Create faces
        faces = create_airplane_faces(rotated_vertices)
        
        # Clear and redraw
        self.ax.clear()
        self.ax.set_xlim([-2, 2])
        self.ax.set_ylim([-2, 2])
        self.ax.set_zlim([-2, 2])
        self.ax.set_xlabel('X (Forward)')
        self.ax.set_ylabel('Y (Right)')
        self.ax.set_zlabel('Z (Up)')
        self.ax.set_title('Flight Orientation Visualization')
        
        # Color airplane parts: fuselage (gray), wings (green/yellow), tail (blue), nose (red)
        colors = ['gray', 'gray', 'gray', 'gray',  # Fuselage
                  'green', 'green',  # Right wing
                  'yellow', 'yellow',  # Left wing
                  'blue',  # Vertical stabilizer
                  'cyan', 'cyan',  # Horizontal stabilizers
                  'red', 'red']  # Nose
        face_collection = Poly3DCollection(faces, alpha=0.8, facecolors=colors, edgecolors='black', linewidths=1.5)
        self.ax.add_collection3d(face_collection)
        
        # Draw coordinate axes
        axis_length = 1.5
        self.ax.quiver(0, 0, 0, axis_length, 0, 0, color='r', arrow_length_ratio=0.1, linewidth=2)
        self.ax.quiver(0, 0, 0, 0, axis_length, 0, color='g', arrow_length_ratio=0.1, linewidth=2)
        self.ax.quiver(0, 0, 0, 0, 0, axis_length, color='b', arrow_length_ratio=0.1, linewidth=2)
        
        return self.ax,
    
    def on_key(self, event):
        """Handle key press events."""
        if event.key == 'q':
            print("Quitting...")
            plt.close(self.fig)
            if self.ser is not None and self.ser.is_open:
                self.ser.close()
    
    def run(self):
        """Start the animation."""
        # Connect key press handler
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)
        
        anim = FuncAnimation(self.fig, self.update, interval=50, blit=False)
        plt.show()
        
        # Close serial when done
        if self.ser is not None and self.ser.is_open:
            self.ser.close()

if __name__ == "__main__":
    print("Quaternion Flight Orientation Visualizer")
    print("========================================")
    print("Press 'Q' to quit")
    print()
    
    # You can specify port directly here, or leave as None to select interactively
    # Example: visualizer = QuaternionVisualizer(port='/dev/ttyUSB0', baudrate=115200)
    visualizer = QuaternionVisualizer(port='/dev/ttyUSB0', baudrate=115200)
    visualizer.run()
