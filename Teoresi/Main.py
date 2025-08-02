
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
        #print(f"Setto In PositionMode:: Velocità: {Speed} Acc: {Acc}")   
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
        #node[Nodo].profileVelocityRPDO(Speed)
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
        val = node[Nodo].FeedBack_StatusWord()
        TxBuffer[0:4] = list(struct.pack('>f', float(val)))
        send(TxBuffer)
    if(funzione==17):
        # bytes_float = bytes(RxBuffer[14:18])
        # Angolo=struct.unpack('>f', bytes_float)[0]
        # node[Nodo].Set_target_position(Angolo)
        statusWord = float((node[Nodo].FeedBack_StatusWord()))
        float_bytes = struct.pack('>f', statusWord)
        TxBuffer[0:4]=list(float_bytes)
        send(TxBuffer)
    if(funzione==18):
        val = node[Nodo].EncoderValue()
        TxBuffer[0:4] = list(struct.pack('>f', val))
        print(f"Valore dell'encoder: {val}")
        send(TxBuffer)     
    if(funzione==19):
        # val = node[Nodo].VelocityValue()
        # TxBuffer[0:4] = list(struct.pack('>f', val if val is not None else 0.0))
        # send(TxBuffer)
        SpeedValue=float(node[Nodo].VelocityValue())
        float_bytes = struct.pack('>f', SpeedValue)
        TxBuffer[0:4]=list(float_bytes)
        send(TxBuffer)

def Init (ID, offset):
    node[ID-1]=Auxind(ID, netWork, Resolution[ID-1], offset)

def spinner():
    while True:
        netWork.EncoderValue_Request(node)
        time.sleep(0.001)

def thread():
    thread = threading.Thread(target=spinner)
    thread.daemon = True  # il thread si chiude quando il programma principale termina
    thread.start()

while True:

    data, addr = sock_recv.recvfrom(30)
    RxBuffer = list(data)
    print(f"Ricevuto da {addr}: {RxBuffer}")


    if RxBuffer[1] == 254:
        NetWork = Network()
        netWork = NetWork.GetNetwork()

    elif (RxBuffer[2] == 255):
        bytes_float = bytes(RxBuffer[10:14])
        offset=struct.unpack('>f', bytes_float)[0]
        Init(RxBuffer[1], offset)

    else:
        NodeId = RxBuffer[1]
        #node[NodeId-1].setFunction(RxBuffer[3])
        start1=time.monotonic()
        SendCanFrame(NodeId-1, RxBuffer[3])
        end1 = time.monotonic()
        elapsed = (end1-start1)*1000
        print(f"Tempo impiegato singolo :: {elapsed} millisecondi")




