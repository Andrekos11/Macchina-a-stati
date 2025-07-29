
import time
import math
import socket
import struct
M_PI = math.pi

TxBuffer = [0xAA, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] #buffer di invio 18 byte
#Byte 1 : 0 : 0xAA = Header
#Byte 2 : 1 : Nodo ID
#Byte 3 : 2 : NMT
#Byte 4 : 3 : Funzione
#Byte 5 : 4 : Event
#Byte 6 : 5 : Mode
#Byte 7-10 : / : Float Velocity
#Byte 11-14 : / : Float Acceleration
#Byte 15-18 : / : Float Angle





ControlWord = {
        "SHUT_DOWN": 0x06,
        "SWITCH_ON": 0x07,
        "ENABLE_OPERATION": 0x0F,
        "CLEAR_BIT": 0x2F,
        "ENABLE_POS_RELATIVE": 0x7F,
        "ENABLE_POS_ABSOLUTE": 0x3F,
        "START_HOMING": 0x1F,
        "DISABLE_VOLTAGE": 0x00,
        "QUICK_STOP": 0x02,
        "DISABLE_OPERATION": 0x07,
        "FAULT_RESET": 0X0080,
    }

ModesOfOperation = {
        "PROFILE_VELOCITY": 0x03,
        "PROFILE_POSITION": 0x01,
        "HOMING": 0x06,
    }
NMT = {
        "PreOperetional": 0x7F,
        "Operetional": 0x05,
        "Stopped": 0x04,
        "Initializing" : 0x00
    }





# IP e porte
TEO_ip = "192.168.137.130"  # IP del Teoresi
port_send = 5005              # Porta su cui Python invia 
port_recv = 5006              # Porta su cui Python riceve

# Socket per inviare dati
sock_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Socket per ricevere dati
sock_recv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_recv.bind(("0.0.0.0", port_recv))




def send(Buffer):
    #spara su ethernet
    msg = bytes(Buffer) 
    sock_send.sendto(msg, (TEO_ip, port_send))
    #print(f"Inviato: {Buffer}")

def FDBack():
    #ricevi su ethernet
    msg = sock_recv.recvfrom(1024)
    Buffer = list(msg[:len(msg)])
    #print(f"Ricevuto: {Buffer}")
    return Buffer

def FDBack2():
    RxBuffer, _ = sock_recv.recvfrom(1024)
    #RxBuffer = list(buffer[:len(buffer)])
    Bytes = RxBuffer[0:4]
    valore = struct.unpack('>f', Bytes)[0]
    
    return valore
    




def InitNetwork():    
        TxBuffer = [0xAA, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255]
        send(TxBuffer)
        while FDBack() is None:

            return FDBack()





class Auxind:

    def __init__(self, NodeId, resolution, offset):   
        self.Nodo = NodeId
        TxBuffer = [0xAA, NodeId, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255]
        float_bytes = struct.pack('>f', offset) #big endian
        TxBuffer[10:14] = list(float_bytes)
        send(TxBuffer)
        while True:
            if FDBack() !=15:
                print("Init eseguito")
                return None
            else:
                print("waiting")
        
    #1    
    def SetVelocityMode(self):
        TxBuffer[0] = 0xAA
        TxBuffer[1] = self.Nodo
        TxBuffer[2] = NMT["Operetional"]
        TxBuffer[3] = 1
        TxBuffer[4] = ControlWord["ENABLE_OPERATION"]
        TxBuffer[5] = ModesOfOperation["PROFILE_VELOCITY"]
        speed = 0.0
        float_bytes = struct.pack('>f', speed) #big endian
        TxBuffer[6:10] = list(float_bytes)
        Acc = 0.08
        float_bytes = struct.pack('>f', Acc) #big endian
        TxBuffer[10:14] = list(float_bytes)
        Angle = 0.0
        float_bytes = struct.pack('>f', Angle) #big endian
        TxBuffer[14:18] = list(float_bytes)
        send(TxBuffer)

    #2
    def SetPositionMode(self, Speed, acceleration):
        TxBuffer[0] = 0xAA
        TxBuffer[1] = self.Nodo
        TxBuffer[2] = NMT["Operetional"]
        TxBuffer[3] = 2
        TxBuffer[4] = ControlWord["ENABLE_OPERATION"]
        TxBuffer[5] = ModesOfOperation["PROFILE_POSITION"]
        float_bytes = struct.pack('>f', Speed) #big endian
        TxBuffer[6:10] = list(float_bytes)
        float_bytes = struct.pack('>f', acceleration) #big endian
        TxBuffer[10:14] = list(float_bytes)      
        send(TxBuffer)

    #3
    def HomingModeCW(self):
        TxBuffer[0] = 0xAA
        TxBuffer[1] = self.Nodo
        TxBuffer[2] = NMT["Operetional"]
        TxBuffer[3] = 3
        TxBuffer[4] = ControlWord["ENABLE_OPERATION"]
        TxBuffer[5] = ModesOfOperation["HOMING"]      
        send(TxBuffer)

    #4
    def HomingModeCCW(self):
        TxBuffer[0] = 0xAA
        TxBuffer[1] = self.Nodo
        TxBuffer[2] = NMT["Operetional"]
        TxBuffer[3] = 4
        TxBuffer[4] = ControlWord["ENABLE_OPERATION"]
        TxBuffer[5] = ModesOfOperation["HOMING"]      
        send(TxBuffer)

    #5
    def SetEncoderToZero (self):
        TxBuffer[0] = 0xAA
        TxBuffer[1] = self.Nodo
        TxBuffer[2] = NMT["Operetional"]
        TxBuffer[3] = 5
        TxBuffer[4] = ControlWord["ENABLE_OPERATION"]
        TxBuffer[5] = ModesOfOperation["HOMING"]      
        send(TxBuffer)

    #6
    def ProfileVelocity(self, Speed, Acc, MaxVel):
        TxBuffer[0] = 0xAA
        TxBuffer[1] = self.Nodo
        TxBuffer[2] = NMT["Operetional"]
        TxBuffer[3] = 6
        TxBuffer[4] = ControlWord["ENABLE_OPERATION"]
        TxBuffer[5] = ModesOfOperation["PROFILE_VELOCITY"]
        float_bytes = struct.pack('>f', Speed) #big endian
        TxBuffer[6:10] = list(float_bytes)
        float_bytes = struct.pack('>f', Acc) #big endian
        TxBuffer[10:14] = list(float_bytes)
        MaxVel= 2
        float_bytes = struct.pack('>f', MaxVel) #big endian
        TxBuffer[14:18] = list(float_bytes)      
        send(TxBuffer)        

    #7
    def ProfilePositionRelative(self, Angle):
        TxBuffer[0] = 0xAA
        TxBuffer[1] = self.Nodo
        TxBuffer[2] = NMT["Operetional"]
        TxBuffer[3] = 7
        TxBuffer[4] = ControlWord["ENABLE_OPERATION"]
        TxBuffer[5] = ModesOfOperation["PROFILE_POSITION"]
        float_bytes = struct.pack('>f', Angle) #big endian
        TxBuffer[14:18] = list(float_bytes)       
        send(TxBuffer)
       
 
        

    #8
    def ProfilePositionAbsolute(self, Angle):
        TxBuffer[0] = 0xAA
        TxBuffer[1] = self.Nodo
        TxBuffer[2] = NMT["Operetional"]
        TxBuffer[3] = 8
        TxBuffer[4] = ControlWord["ENABLE_OPERATION"]
        TxBuffer[5] = ModesOfOperation["PROFILE_POSITION"]
        float_bytes = struct.pack('>f', Angle) #big endian
        TxBuffer[14:18] = list(float_bytes)       
        send(TxBuffer)

    #9
    def Shutdown(self):
        TxBuffer[0] = 0xAA
        TxBuffer[1] = self.Nodo
        TxBuffer[2] = NMT["Operetional"]
        TxBuffer[3] = 9
        TxBuffer[4] = ControlWord["SHUT_DOWN"]      
        send(TxBuffer)

    #10
    def Disable_Voltage(self):
        TxBuffer[0] = 0xAA
        TxBuffer[1] = self.Nodo
        TxBuffer[2] = NMT["Operetional"]
        TxBuffer[3] = 10
        TxBuffer[4] = ControlWord["DISABLE_VOLTAGE"]      
        send(TxBuffer)
    #11
    def Switch_On(self):
        TxBuffer[0] = 0xAA
        TxBuffer[1] = self.Nodo
        TxBuffer[2] = NMT["Operetional"]
        TxBuffer[3] = 11
        TxBuffer[4] = ControlWord["SWITCH_ON"]      
        send(TxBuffer)
    #12
    def Disable_Operation(self):
        TxBuffer[0] = 0xAA
        TxBuffer[1] = self.Nodo
        TxBuffer[2] = NMT["Operetional"]
        TxBuffer[3] = 12
        TxBuffer[4] = ControlWord["DISABLE_OPERATION"]      
        send(TxBuffer)
    #13
    def Enable_Operation(self):
        TxBuffer[0] = 0xAA
        TxBuffer[1] = self.Nodo
        TxBuffer[2] = NMT["Operetional"]
        TxBuffer[3] = 13
        TxBuffer[4] = ControlWord["ENABLE_OPERATION"]      
        send(TxBuffer)
    #14
    def Fault_Reset(self):
        TxBuffer[0] = 0xAA
        TxBuffer[1] = self.Nodo
        TxBuffer[2] = NMT["Operetional"]
        TxBuffer[3] = 14
        TxBuffer[4] = ControlWord["FAULT_RESET"]      
        send(TxBuffer)
    #15
    def Quick_Stop(self):
        TxBuffer[0] = 0xAA
        TxBuffer[1] = self.Nodo
        TxBuffer[2] = NMT["Operetional"]
        TxBuffer[3] = 15
        TxBuffer[4] = ControlWord["QUICK_STOP"]      
        send(TxBuffer)
    
    #16
    def FeedBack_StatusWord(self):
        TxBuffer[0] = 0xAA
        TxBuffer[1] = self.Nodo
        TxBuffer[3] = 16     
        send(TxBuffer)
        return FDBack2()

    #17
    def Set_target_position(self, Angle):
        TxBuffer[0] = 0xAA
        TxBuffer[1] = self.Nodo
        TxBuffer[3] = 17
        float_bytes = struct.pack('>f', Angle) #big endian
        TxBuffer[14:18] = list(float_bytes)       
        send(TxBuffer)

    #18
    def EncoderValue(self):
        TxBuffer[0] = 0xAA
        TxBuffer[1] = self.Nodo
        TxBuffer[3] = 18   
        send(TxBuffer)
        return FDBack2()
        

    #19
    def VelocityValue(self):
        TxBuffer[0] = 0xAA
        TxBuffer[1] = self.Nodo
        TxBuffer[3] = 19
        send(TxBuffer)
        return FDBack2()
    

    