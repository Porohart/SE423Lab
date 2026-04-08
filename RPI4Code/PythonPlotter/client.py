import socket
import time
import numpy as np

HOST = '127.0.0.1'
PORT = 10001

def start_server():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    try:
        server_socket.bind((HOST, PORT))
        server_socket.listen(1)
    except Exception as e:
        print(f"Fatal binding error: {e}")
        return

    while True:
        print("Waiting for connection from client...")
        client_conn, client_addr = server_socket.accept()
        print(f"Client connected from {client_addr}")
        
        p_k = np.zeros(8)
        
        sigma = 0.2 
        
        try:
            while True:
                w_k = np.random.normal(0, sigma, 8)
                
                p_k = p_k + w_k
                
                state_vector = p_k
                
                msg = f"{' '.join(f'{x:.6f}' for x in state_vector)}\r\n"
                
                client_conn.sendall(msg.encode('ascii'))
                
                time.sleep(0.05)
                
        except (ConnectionResetError, BrokenPipeError):
            print("Client disconnected.")
        finally:
            client_conn.close()

if __name__ == "__main__":
    start_server()