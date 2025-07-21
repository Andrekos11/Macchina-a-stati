from LibreriaAuxind import*


Resolution = [(12800*100), (12800*120), (12800*100), (12800*100), (12800*100), (12800*100), (12800*100)]
RxBuffer = [0xAA, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] #Buffer di ricezione 9Byte min

NodeId = 1
node = {}
TxBuffer = [0, 0, 0, 0]
def SendCanFrame(Nodo, funzione):
    if(funzione==1):
        node[Nodo].SetVelocityMode()
    if(funzione==2):
        bytes_float = bytes(RxBuffer[6:10])
        Speed=struct.unpack('>f', bytes_float)[0] 
        bytes_float = bytes(RxBuffer[10:14])
        Acc=struct.unpack('>f', bytes_float)[0]  
        print(f"Setto In PositionMode:: Velocità: {Speed} Acc: {Acc}")   
        node[Nodo].SetPositionMode(Speed, Acc)
    if(funzione==3):
        node[Nodo].HomingModeCW()
    if(funzione==4):
        node[Nodo].HomingModeCCW()
    if(funzione==5):
        node[Nodo].SetEncoderToZero()
    if(funzione==6):
        bytes_float = bytes(RxBuffer[6:10])
        Speed=struct.unpack('>f', bytes_float)[0]
        bytes_float = bytes(RxBuffer[10:14])
        Acc=struct.unpack('>f', bytes_float)[0]
        bytes_float = bytes(RxBuffer[14:18])
        MaxVel=struct.unpack('>f', bytes_float)[0]        
        node[Nodo].ProfileVelocity(Speed, Acc, MaxVel)
    if(funzione==7):
        bytes_float = bytes(RxBuffer[14:18])
        Angolo=struct.unpack('>f', bytes_float)[0]
        node[Nodo].ProfilePositionRelative(Angolo)
    if(funzione==8):
        bytes_float = bytes(RxBuffer[14:18])
        Angolo=struct.unpack('>f', bytes_float)[0]
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
        statusWord = float((node[Nodo].FeedBack_StatusWord()))
        float_bytes = struct.pack('>f', statusWord)
        TxBuffer[0:4]=list(float_bytes)
        send(TxBuffer)
    if(funzione==17):
        bytes_float = bytes(RxBuffer[14:18])
        Angolo=struct.unpack('>f', bytes_float)[0]
        node[Nodo].Set_target_position(Angolo)
    if(funzione==18):
        EncoderValue= float((node[Nodo].EncoderValue()) )   
        float_bytes = struct.pack('>f', EncoderValue)
        TxBuffer[0:4]=list(float_bytes)
        print(TxBuffer)
        send(TxBuffer)
    if(funzione==19):
        SpeedValue=float(node[Nodo].VelocityValue())
        float_bytes = struct.pack('>f', SpeedValue)
        TxBuffer[0:4]=list(float_bytes)
        print(TxBuffer)
        send(TxBuffer)

def Init (ID, offset):
    node[ID-1]=Auxind(ID, NetWork, Resolution[ID-1], offset)
     

while True:
    data, addr = sock_recv.recvfrom(1024)
    if len(data) == 18:
        RxBuffer = list(data)
        print(f"Ricevuto da {addr}: {RxBuffer}")
    else: 
        print("Pacchetto minore di 18 bytes")

    if RxBuffer == [0xAA, 256, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255]:
        NetWork = InitNetwork()

    elif (RxBuffer[0] == 0xAA and RxBuffer[2] == 255 and RxBuffer[3] == 255 and RxBuffer[4] == 255 and RxBuffer[5] == 255):
        bytes_float = bytes(RxBuffer[10:14])
        offset=struct.unpack('>f', bytes_float)[0]
        Init(RxBuffer[1], offset)

    else:
        print("lettura pacchetto di evento")
        NodeId = RxBuffer[1]
        print("Nodo: ", NodeId)
        node[NodeId-1].setFunction(RxBuffer[3])
        SendCanFrame(NodeId-1, RxBuffer[3])




