import struct
import socket



# IP e porte
NUC_ip = "192.168.1.140"  # IP del NUC
port_send = 5006              
port_recv = 5005              

# Socket per inviare dati a LabVIEW
sock_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Socket per ricevere dati da LabVIEW
sock_recv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_recv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock_recv.bind(("0.0.0.0", port_recv))


def send(Buffer):
    msg = bytes(Buffer)
    sock_send.sendto(msg, (NUC_ip, port_send))
    #print(f"Inviato: {Buffer}")

def Receive():
    data, addr = sock_recv.recvfrom(1024)
    RxBuffer = list(data[:len(data)])
    #print(f"Ricevuto da {addr}: {RxBuffer}")
    return RxBuffer
