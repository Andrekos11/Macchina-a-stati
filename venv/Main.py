from LibreriaAuxind import*
import struct
import socket

# IP e porte
labview_ip = "192.168.1.100"  # IP di LabVIEW
port_send = 5005              # Porta su cui Python invia a LabVIEW
port_recv = 5006              # Porta su cui Python riceve da LabVIEW

# Socket per inviare dati a LabVIEW
sock_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Socket per ricevere dati da LabVIEW
sock_recv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_recv.bind(("0.0.0.0", port_recv))

def Invia(Buffer):
    # Invia un float come 4 byte
    msg = struct.pack('f', Buffer)
    sock_send.sendto(msg, (labview_ip, port_send))
    print(f"Inviato: {Buffer}")

def Ricevi():
    data, addr = sock_recv.recvfrom(1024)
    if len(data) == 9:
        buffer = struct.unpack('9B', data)  # 9 unsigned byte
        Buffer = list(buffer)
        return Buffer
    else:
        print(f"Ricevuto {len(data)} byte ma ne aspettavo 9")
        return None

Resolution = [12800*100, 12800*120, 12800*120, 12800*100, 12800*100, 12800*100, 12800*100]
RxBuffer = [0xAA, 0, 0, 0, 0, 0, 0, 0, 0] #Buffer di ricezione 9Byte min

NodeId = 1

def SendCanFrame(Nodo, Speed, Acc, Angolo, funzione):
    if(funzione==1):
        node[Nodo].SetVelocityMode()
    if(funzione==2):
        node[Nodo].SetPositionMode(Speed, Acc)
    if(funzione==3):
        node[Nodo].HomingModeCW()
    if(funzione==4):
        node[Nodo].HomingModeCCW()
    if(funzione==5):
        node[Nodo].SetEncoderToZero()
    if(funzione==6):
        node[Nodo].ProfileVelocity(Speed)
    if(funzione==7):
        node[Nodo].ProfilePositionRelative(Angolo)
    if(funzione==8):
        node[Nodo].ProfilePositionAbsolute(Angolo)
    if(funzione==9):
        node[Nodo].Shutdown()
    if(funzione==10):
        node[Nodo].Disable_Voltage()
    if(funzione==11):
        node[Nodo].Switch_On()
    if(funzione==12):
        node[Nodo].Disable_Operation()
    if(funzione==13):
        node[Nodo].Enable_Operation()
    if(funzione==14):
        node[Nodo].Fault_Reset()
    if(funzione==15):
        node[Nodo].Quick_Stop()
    if(funzione==16):
        node[Nodo].FeedBack_StatusWord()
    if(funzione==17):
        node[Nodo].Set_target_position(Angolo)
    if(funzione==18):
        node[Nodo].EncoderValue()
    if(funzione==19):
        node[Nodo].VelocityValue()
    

while True:
    data, addr = sock_recv.recvfrom(1024)
    RxBuffer = list(data[:9])
    print(f"Ricevuto da {addr}: {RxBuffer}")

    #Spacchetto il buffer
    for x in range(9):
        if RxBuffer[x]==0xAA: #Header
            if RxBuffer == [0xAA, 255 , 255, 255, 255, 255, 255, 255, 255]:
                NetWork = InitNetwork()
                node = [Auxind(i, NetWork, Resolution[i]) for i in range(8)] #Array di motori
    
            #spacchetto il msg 

            #byte1 assegno numero nodo
            NodeId = RxBuffer[x+1]
            #byte2 NMT
            node[NodeId].NMT_SetRequest(RxBuffer[x+2])
            #byte3, 6, 7 ,8 
            node[NodeId].setFunzione(RxBuffer[x+3], RxBuffer[x+5], RxBuffer[x+6], RxBuffer[x+7], RxBuffer[x+8])
            #byte4 evento
            node[NodeId].Event_SetRequest(RxBuffer[x+1])

            SendCanFrame(NodeId, RxBuffer[x+6], RxBuffer[x+7], RxBuffer[x+8], RxBuffer[x+3])
            #macchina a stati?

            # if((node[NodeId].NMT_ReadRequest() == node[NodeId].NMT_ReadRequest()) and ):



