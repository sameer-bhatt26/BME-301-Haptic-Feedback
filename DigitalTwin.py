import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import pyaudio

# ------------------------------------
# Parameters
# ------------------------------------
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 44100
CHUNK = 4096               # Larger chunk = better FFT resolution
RECORD_SECONDS = 3000
NUM_BANDS = list(range(18))
matching_list = list(range(18))

LOW_FREQ = 50
HIGH_FREQ = 8000
THRESHOLD = 2e5            # Adjust depending on mic sensitivity

# ------------------------------------
# Logarithmic frequency bands
# ------------------------------------
fft_freqs = np.fft.fftfreq(CHUNK, 1.0 / RATE)[:CHUNK // 2]

log_edges_hz = np.logspace(np.log10(LOW_FREQ),
                           np.log10(HIGH_FREQ),
                           len(NUM_BANDS) + 1)

band_bin_indices = []
for i in NUM_BANDS:
    low_edge = log_edges_hz[i]
    high_edge = log_edges_hz[i + 1]
    bins = np.where((fft_freqs >= low_edge) &
                    (fft_freqs < high_edge))[0]
    band_bin_indices.append(bins)

# ------------------------------------
# FFT window
# ------------------------------------
window = np.hanning(CHUNK)

# ------------------------------------
# Create 3D Matplotlib Cylinder Display
# ------------------------------------
plt.ion()
fig = plt.figure(figsize=(7, 6))
ax = fig.add_subplot(111, projection='3d')
ax.set_axis_off()

R = 2.0                     # cylinder radius
rows = 6
cols = 3
tile_h = 1.0                # height of each tile
tile_arc = np.deg2rad(30)   # small arc size for each tile (12° wide)
z_levels = np.linspace(0, rows * tile_h, rows + 1)
theta_centers = np.linspace(0, 2 * np.pi, cols, endpoint=False)

tiles = []

def make_tile(center_theta, width_theta, z1, z2, color):
    """Makes a small rectangle glued to the cylinder surface."""
    t1 = center_theta - width_theta / 2
    t2 = center_theta + width_theta / 2

    x = [R * np.cos(t1), R * np.cos(t2),
         R * np.cos(t2), R * np.cos(t1)]
    y = [R * np.sin(t1), R * np.sin(t2),
         R * np.sin(t2), R * np.sin(t1)]
    z = [z1, z1, z2, z2]

    verts = [list(zip(x, y, z))]
    poly = Poly3DCollection(verts, color=color, edgecolor="black", linewidths=1.0)
    ax.add_collection3d(poly)
    return poly


# Draw the base cylinder (smooth mesh)
theta = np.linspace(0, 2 * np.pi, 200)
z = np.linspace(0, rows * tile_h, 50)
theta_grid, z_grid = np.meshgrid(theta, z)
x_cyl = R * np.cos(theta_grid)
y_cyl = R * np.sin(theta_grid)

ax.plot_surface(x_cyl, y_cyl, z_grid, color="lightgray", alpha=0.2)

# Build 3×6 tiles around the cylinder
idx = 0
for r in range(rows):
    z1 = z_levels[r]
    z2 = z_levels[r + 1]
    for c in range(cols):
        center_theta = theta_centers[c]
        poly = make_tile(center_theta, tile_arc, z1, z2, "lightgray")
        tiles.append(poly)
        idx += 1

ax.set_xlim([-R - 1, R + 1])
ax.set_ylim([-R - 1, R + 1])
ax.set_zlim([0, z_levels[-1]])
ax.view_init(elev=15, azim=60)

plt.show()

# ------------------------------------
# Initialize microphone
# ------------------------------------
p = pyaudio.PyAudio()

stream = p.open(format=FORMAT,
                channels=CHANNELS,
                rate=RATE,
                input=True,
                frames_per_buffer=CHUNK,
                input_device_index=1)

print("Recording…")

# ------------------------------------
# Main processing loop
# ------------------------------------
num_iters = int(RATE / CHUNK * RECORD_SECONDS)

for _ in range(num_iters):

    data = stream.read(CHUNK)
    data_np = np.frombuffer(data, dtype=np.int16)

    windowed = data_np * window

    fft_vals = np.abs(np.fft.fft(windowed))[:CHUNK // 2]

    band_sums = []
    for bins in band_bin_indices:
        if len(bins) > 0:
            band_sums.append(np.sum(fft_vals[bins]))
        else:
            band_sums.append(0)

    # Update tile colors
    for i in NUM_BANDS:
        curr_i = matching_list[i]
        if band_sums[i] > THRESHOLD:
            tiles[curr_i].set_facecolor("red")
        else:
            tiles[curr_i].set_facecolor("lightgray")

    fig.canvas.draw()
    fig.canvas.flush_events()

print("Done.")

stream.stop_stream()
stream.close()
p.terminate()
