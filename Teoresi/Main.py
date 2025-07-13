from LibreriaAuxind import*


Resolution = [(12800*100), (12800*120), (12800*120), 12800*100, 12800*100, 12800*100, 12800*100]
RxBuffer = [0xAA, 0, 0, 0, 0, 0, 0, 0, 0] #Buffer di ricezione 9Byte min

NodeId = 1
node = {}

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
        node[Nodo].ProfileVelocity(Speed)
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
        send(node[Nodo].FeedBack_StatusWord())
    if(funzione==17):
        bytes_float = bytes(RxBuffer[14:18])
        Angolo=struct.unpack('>f', bytes_float)[0]
        node[Nodo].Set_target_position(Angolo)
    if(funzione==18):
        send(node[Nodo].EncoderValue())      
    if(funzione==19):
        send(node[Nodo].VelocityValue())

def Init (ID):
    node[ID-1]=Auxind(ID, NetWork, Resolution[ID-1])
     

while True:
    data, addr = sock_recv.recvfrom(1024)
    if len(data) == 18:
        RxBuffer = list(data)
        print(f"Ricevuto da {addr}: {RxBuffer}")
    else: 
        print("Pacchetto minore di 18 bytes")

    if RxBuffer == [0xAA, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255]:
        NetWork = InitNetwork()
        #
        # print(NetWork)

    elif (RxBuffer[0] == 0xAA and RxBuffer[2] == 255 and RxBuffer[3] == 255 and RxBuffer[4] == 255 and RxBuffer[5] == 255 and RxBuffer[6] == 255 and RxBuffer[7] == 255 and RxBuffer[8] == 255):
        #node[0] = Auxind((1), NetWork, Resolution[0])
        Init(RxBuffer[1])
        #node = [Auxind(i, NetWork, Resolution[i]) for i in range(7)] #Array di motori

    #spacchetto il msg 
    #elif RxBuffer != [0xAA, 255 , 255, 255, 255, 255, 255, 255, 255] and RxBuffer != [171, 255 , 255, 255, 255, 255, 255, 255, 255]: 
    else:
        print("lettura pacchetto di evento")
        #byte1 assegno numero nodo
        NodeId = RxBuffer[1]
        print("Nodo: ", NodeId)
        #byte2 NMT
        #node[NodeId].NMT_SetRequest(RxBuffer[x+2])
        #byte3, 6, 7 ,8 
        #node[(NodeId-1)].setFunzione(RxBuffer[3], RxBuffer[5], RxBuffer[6], RxBuffer[7], RxBuffer[8])
        #byte4 evento
        #node[NodeId].Event_SetRequest(RxBuffer[x+1])
        node[NodeId-1].setFunction(RxBuffer[3])
        SendCanFrame(NodeId-1, RxBuffer[3])
        #macchina a stati?

        # if((node[NodeId].NMT_ReadRequest() == node[NodeId].NMT_ReadRequest()) and ):



