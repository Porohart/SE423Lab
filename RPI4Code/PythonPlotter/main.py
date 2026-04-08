from functools import partial
import socket
import warnings
from matplotlib.gridspec import GridSpec
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import patches
import numpy as np

from matplotlib.widgets import CheckButtons, TextBox, Button

HOST = '127.0.0.1'
PORT = 10001

SIZE_OF_RECT = 20

MAX_NUM_RECTS = 100

MARGIN = 50

DPI = 300

SHOW_PLOT = True
SHOW_SQUARES = True

def init_socket(host, port):
    socket_client = None
    
    try:
        socket_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        socket_client.connect((host, port))
        socket_client.settimeout(1.0)
        print("Connected to server at {}:{}".format(host, port))
    except:
        print("Failed to connect to server at {}:{}".format(host, port))
        exit(1)
    
    return socket_client

buffer = ''

rectangles = []


def set_axis_limits(ax):
    data = []

    for p in ax.patches:
        data.extend(p.get_bbox().get_points())

    data = np.array(data)

    x_min, y_min = np.min(data, axis=0)
    x_max, y_max = np.max(data, axis=0)

    ax.set_xlim(x_min - MARGIN, x_max + MARGIN)
    ax.set_ylim(y_min - MARGIN, y_max + MARGIN)
    

def fetch_and_update(frame, socket_client, ax, input_text_boxes, figDraw, lineObject):
    global buffer
    global rectangles
    
    try:
        data = socket_client.recv(4096).decode('ascii')
        if data:
            buffer += data
            while '\r\n' in buffer:
                line, buffer = buffer.split('\r\n', 1)
                if line:
                    floats = list(map(float, line.split(' ')))
                    
                    if len(floats) == 8:
                        x, y = floats[0], floats[1]
                        
                        for i, val in enumerate(floats):
                            input_text_boxes[i].set_text(f"{val:.2f}")
                            
                        figDraw.canvas.draw_idle()
                        
                        x = (x + 8) * 50
                        y = (12 - y) * 50
                        
                        x = int(x)
                        y = int(y)

                        angle = floats[2]
                        
                        x_min = x - SIZE_OF_RECT
                        y_min = y - SIZE_OF_RECT
                        
                        width = SIZE_OF_RECT * 2
                        height = SIZE_OF_RECT * 2
                        
                        rect = patches.Rectangle(
                            (x_min, y_min), width, height, angle=np.rad2deg(angle),
                            linewidth=1, edgecolor='black', facecolor='cyan', alpha=0.6
                        )
                        
                        ax.add_patch(rect)
                        
                        if SHOW_SQUARES:
                            [rect.set_visible(True) for rect in ax.patches]
                        else:
                            [rect.set_visible(False) for rect in ax.patches]
                        
                        while len(ax.patches) > MAX_NUM_RECTS:
                            ax.patches[0].remove()
                            
                        if SHOW_PLOT:
                            data = [p.get_center() for p in ax.patches]
                                
                            data = np.array(data)
                            lineObject.set_data(data[:, 0], data[:, 1])
                            lineObject.set_alpha(0.5)
                        else:
                            lineObject.set_data([], [])
                                
                        set_axis_limits(ax)
                        # ax.patches = ax.patches[-MAX_NUM_RECTS:]
                    else:
                        print(f"Received malformed line: {line}")
    except socket.timeout:
        pass
    except BlockingIOError:
        pass
    except KeyboardInterrupt:
        print("Interrupted by user. Exiting.")
        exit(0)
    
    return ax.patches

def transmit( socket_client, text, event):
    data = []
    
    for tb in text:
        try:
            val = float(tb.text)
            data.append(val)
        except ValueError:
            data.append(0.0)
            tb.set_val('0.0')

    text = " ".join(f"{d:.6f}" for d in data)
    
    msg = b"\xFD" + text.encode("ascii") + b"\xFF"
    
    print("Transmitting: {}".format(msg))
    socket_client.sendall(msg)
    
def parse_args():
    import argparse
    
    parser = argparse.ArgumentParser(description="Real-time robot position plotter")
    parser.add_argument('--host', type=str, default=HOST, help='Server host (default: localhost)')
    parser.add_argument('--port', type=int, default=PORT, help='Server port (default: 10001)')
    parser.add_argument('--dpi', type=int, default=DPI, help='DPI for saved plot (default: 300)')
    
    return parser.parse_args()

def main():
    args = parse_args()
    host = args.host
    port = args.port
    global DPI
    DPI = args.dpi
    
    socket_client = init_socket(host, port)
    
    fig, ax = plt.subplots(figsize=(8, 8))
    
    figData = plt.figure(figsize=(10, 3))
    
    
    ax.set_title("Robot Position Visualization", fontsize=14, fontweight='bold')
    ax.set_xlabel("X Coordinate", fontsize=12)
    ax.set_ylabel("Y Coordinate", fontsize=12)
    ax.grid(True, linestyle='--', alpha=0.6)
    
    ax.set_aspect('equal', adjustable='box')
    # ax.set_aspect('equal', adjustable='datalim')
    
    num_cols = 8
    
    gs = GridSpec(3, num_cols + 1, figure=figData, 
                  wspace=0.3, hspace=0.6, 
                  left=0.05, right=0.95, top=0.85, bottom=0.15)
    
    input_text_boxes = []
    
    for i in range(8):
        ax_box = figData.add_subplot(gs[0, i])
        ax_box.axis('off')
        
        text_box = ax_box.text(
            0.5, 0.5, '0.00', 
            transform=ax_box.transAxes, 
            ha='center', va='center', 
            fontsize=10, 
            bbox=dict(facecolor='white', edgecolor='black', alpha=0.8)
        )
        input_text_boxes.append(text_box)
    
    text_boxes = []
    
    subbmit_function = partial(transmit, socket_client, text_boxes)
    
    for i in range(8):
        ax_box = figData.add_subplot(gs[1, i])
        text_box = TextBox(ax_box, f'', initial='0.0')
        text_box.on_submit(subbmit_function)
        text_boxes.append(text_box)
        
    ax_btn = figData.add_subplot(gs[1, num_cols])
    btn_send = Button(ax_btn, 'Send', color='lightgreen', hovercolor='palegreen')
    
    btn_send.on_clicked(subbmit_function)
    
    ax_btn = figData.add_subplot(gs[2, 1])
    text2 = TextBox(ax_btn, 'History length', initial=str(MAX_NUM_RECTS))
    
    def update_max_num_rects(val):
        global MAX_NUM_RECTS
        MAX_NUM_RECTS = int(val)
    text2.on_submit(update_max_num_rects)
    
    ax_btn = figData.add_subplot(gs[2, 4])
    text3 = TextBox(ax_btn, 'Rectangle size', initial=str(SIZE_OF_RECT))
    
    def update_size_of_rect(val):
        global SIZE_OF_RECT
        SIZE_OF_RECT = float(val)
    text3.on_submit(update_size_of_rect)
    
    
    ax_btn = figData.add_subplot(gs[2, 6:8])
    enable_line = CheckButtons(ax_btn, ['Show line', "Show squares"], [SHOW_PLOT, SHOW_SQUARES])
    
    def update_show_plot(label):
        global SHOW_PLOT, SHOW_SQUARES
        if label == 'Show line':
            SHOW_PLOT = not SHOW_PLOT
        elif label == 'Show squares':
            SHOW_SQUARES = not SHOW_SQUARES
    enable_line.on_clicked(update_show_plot)
    
    text = ax.text(0.5, 1.05, "Press 'q' to quit and save plot", transform=ax.transAxes, ha='center', fontsize=10)
    
    
    line = ax.plot([], [], 'o-', color='blue', alpha=0.5)[0]
    
    ani = animation.FuncAnimation(fig, fetch_and_update, interval=50, blit=False, cache_frame_data=False, fargs=(
        socket_client,  ax, input_text_boxes, figData, line
    ))
    
    def press(event):
        if event.key == 'q':
            ani.event_source.stop()
            fig.savefig("plot.png", dpi=DPI)
            fig.savefig("plot.pdf")
            print("Plot saved")
            plt.close("all")

    cid = fig.canvas.mpl_connect('key_press_event', press)
    cid2 = figData.canvas.mpl_connect('key_press_event', press)
    
    try:
        plt.show()
    finally:
        socket_client.close()
        plt.close('all')


if __name__ == "__main__":
    with warnings.catch_warnings():
            warnings.filterwarnings("ignore", category=UserWarning, message="Ignoring fixed .*")
            main()
