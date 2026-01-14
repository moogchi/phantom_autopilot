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

def create_box_vertices():
    """Create vertices for a box (cuboid) representing the flight controller."""
    # Box dimensions (width, height, depth)
    w, h, d = 2.0, 0.5, 1.0
    
    # Define the 8 vertices of the box
    vertices = np.array([
        [-w/2, -h/2, -d/2],  # 0
        [ w/2, -h/2, -d/2],  # 1
        [ w/2,  h/2, -d/2],  # 2
        [-w/2,  h/2, -d/2],  # 3
        [-w/2, -h/2,  d/2],  # 4
        [ w/2, -h/2,  d/2],  # 5
        [ w/2,  h/2,  d/2],  # 6
        [-w/2,  h/2,  d/2],  # 7
    ])
    
    return vertices

def create_box_faces(vertices):
    """Define the 6 faces of the box."""
    faces = [
        [vertices[0], vertices[1], vertices[2], vertices[3]],  # bottom
        [vertices[4], vertices[5], vertices[6], vertices[7]],  # top
        [vertices[0], vertices[1], vertices[5], vertices[4]],  # front
        [vertices[2], vertices[3], vertices[7], vertices[6]],  # back
        [vertices[0], vertices[3], vertices[7], vertices[4]],  # left
        [vertices[1], vertices[2], vertices[6], vertices[5]],  # right
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
        
        # Current quaternion
        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0])  # [w, x, y, z]
        
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
        
        # Initialize box
        self.box_vertices = create_box_vertices()
        self.poly = None
        
    def read_quaternion(self):
        """Read quaternion from serial port."""
        if self.ser is None or not self.ser.is_open:
            return None
        
        try:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    # Parse comma-separated values
                    values = [float(v.strip()) for v in line.split(',')]
                    if len(values) == 4:
                        return np.array(values)
        except Exception as e:
            print(f"Error reading serial: {e}")
        
        return None
    
    def update(self, frame):
        """Update function for animation."""
        # Read new quaternion data
        new_quat = self.read_quaternion()
        if new_quat is not None:
            self.quaternion = new_quat
            print(f"Quat: [{self.quaternion[0]:.4f}, {self.quaternion[1]:.4f}, {self.quaternion[2]:.4f}, {self.quaternion[3]:.4f}]")
        
        # Apply rotation to box
        R = quaternion_to_rotation_matrix(self.quaternion)
        rotated_vertices = np.dot(self.box_vertices, R.T)
        
        # Create faces
        faces = create_box_faces(rotated_vertices)
        
        # Clear and redraw
        self.ax.clear()
        self.ax.set_xlim([-2, 2])
        self.ax.set_ylim([-2, 2])
        self.ax.set_zlim([-2, 2])
        self.ax.set_xlabel('X (Forward)')
        self.ax.set_ylabel('Y (Right)')
        self.ax.set_zlabel('Z (Up)')
        self.ax.set_title('Flight Orientation Visualization')
        
        # Color faces differently
        colors = ['cyan', 'yellow', 'red', 'green', 'blue', 'magenta']
        face_collection = Poly3DCollection(faces, alpha=0.7, facecolors=colors, edgecolors='black', linewidths=2)
        self.ax.add_collection3d(face_collection)
        
        # Draw coordinate axes
        axis_length = 1.5
        self.ax.quiver(0, 0, 0, axis_length, 0, 0, color='r', arrow_length_ratio=0.1, linewidth=2)
        self.ax.quiver(0, 0, 0, 0, axis_length, 0, color='g', arrow_length_ratio=0.1, linewidth=2)
        self.ax.quiver(0, 0, 0, 0, 0, axis_length, color='b', arrow_length_ratio=0.1, linewidth=2)
        
        return self.ax,
    
    def run(self):
        """Start the animation."""
        anim = FuncAnimation(self.fig, self.update, interval=50, blit=False)
        plt.show()
        
        # Close serial when done
        if self.ser is not None and self.ser.is_open:
            self.ser.close()

if __name__ == "__main__":
    print("Quaternion Flight Orientation Visualizer")
    print("========================================")
    
    # You can specify port directly here, or leave as None to select interactively
    # Example: visualizer = QuaternionVisualizer(port='/dev/ttyUSB0', baudrate=115200)
    visualizer = QuaternionVisualizer(port='/dev/ttyUSB0', baudrate=115200)
    visualizer.run()
