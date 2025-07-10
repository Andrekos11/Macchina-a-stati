import canopen
import time
import math
M_PI = math.pi


TxBuffer = [0xAA, 0, 0, 0, 0, 0, 0, 0, 0] #buffer di invio

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
        "DISABLE_OPERATION": 0X07,
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

def send():
    #spara su ethernet
    print(TxBuffer)

def InitNetwork():    
        TxBuffer = [0xAA, 255, 255, 255, 255, 255, 255, 255, 255]
        send(TxBuffer)

class Auxind:

    def __init__(self, NodeId):   
        self.Nodo = NodeId
        
    #1    
    def SetVelocityMode(self):
        TxBuffer = [0xAA, self.Nodo, NMT["Operetional"], 1, ControlWord["ENABLE_OPERATION"], ModesOfOperation["PROFILE_VELOCITY"], 10, 2, 0]
        send(TxBuffer)

    #2
    def SetPositionMode(self, Speed, acceleration):
        speed = abs((self.Resolution*Speed)/(2*M_PI))
        acc = abs((acceleration * self.Resolution) / (2 * M_PI))
        TxBuffer = [0xAA, self.Nodo, NMT["Operetional"], 2, ControlWord["SWITCH_ON"], ModesOfOperation["PROFILE_POSITION"], speed, acc, 0]
        send(TxBuffer)

    #3
    def HomingModeCW(self):
        TxBuffer = [0xAA, self.Nodo, NMT["Operetional"], 3, ControlWord["START_HOMING"], ModesOfOperation["HOMING"], 5, 2, 0]
        send(TxBuffer)

    #4
    def HomingModeCCW(self):
        TxBuffer = [0xAA, self.Nodo, NMT["Operetional"], 4, ControlWord["START_HOMING"], ModesOfOperation["HOMING"], 5, 2, 0]
        send(TxBuffer)

    #5
    def SetEncoderToZero (self):

        self.Nodo.sdo[0x6060].raw = ModesOfOperation["HOMING"]
        self.Nodo.sdo.download(0x6098, 0, b'\x23')        #11 antiorario     12 orario
        self.Nodo.sdo[0x6099][1].raw = 15000  # homing speed research
        self.Nodo.sdo[0x6099][2].raw = 15000  # homing speed release
        self.Nodo.sdo[0x609A].raw = 100  # homing acceleration
        self.Nodo.sdo[0x607C].raw = 0x00

        # Shutdown
        self.Nodo.sdo[0x6040].raw = ControlWord["SHUT_DOWN"]
        time.sleep(0.2)

        # Switch on
        self.Nodo.sdo[0x6040].raw = ControlWord["SWITCH_ON"]
        time.sleep(0.2)

        # Attiva il movimento
        self.Nodo.sdo[0x6040].raw = ControlWord["START_HOMING"]  #start homing
        time.sleep(0.2)

    #6
    def ProfileVelocity(self, Speed):
        speed = self.Resolution*Speed/2/M_PI
        TxBuffer[6] = Speed
        TxBuffer[3] = 6
        send(TxBuffer)
        

    #7
    def ProfilePositionRelative(self, Angle):
        gradi= self.Resolution/360*Angle        #gradi= self.Resolution*Angle/2/M_PI
        TxBuffer[8] = gradi
        TxBuffer[3] = 7
        send(TxBuffer)
        

    #8
    def ProfilePositionAbsolute(self, Angle):
        gradi= self.Resolution/360*Angle        #gradi= self.Resolution*Angle/2/M_PI
        TxBuffer[8] = gradi
        TxBuffer[3] = 8
        send(TxBuffer)

    #9
    def Shutdown(self):
        TxBuffer[4] = ControlWord["SHUT_DOWN"]
        TxBuffer[3] = 9
        send(TxBuffer)

    #10
    def Disable_Voltage(self):
        TxBuffer[4] = ControlWord["DISABLE_VOLTAGE"]
        TxBuffer[3] = 10
        send(TxBuffer)
    #11
    def Switch_On(self):
        TxBuffer[4] = ControlWord["SWITCH_ON"]
        TxBuffer[3] = 11
        send(TxBuffer)
    #12
    def Disable_Operation(self):
        TxBuffer[4] = ControlWord["DISABLE_OPERATION"]
        TxBuffer[3] = 12
        send(TxBuffer)
    #13
    def Enable_Operation(self):
        TxBuffer[4] = ControlWord["ENABLE_OPERATION"]
        TxBuffer[3] = 13
        send(TxBuffer)
    #14
    def Fault_Reset(self):
        TxBuffer[4] = ControlWord["FAULT_RESET"]
        TxBuffer[3] = 14
        send(TxBuffer)
    #15
    def Quick_Stop(self):
        TxBuffer[4] = ControlWord["QUICK_STOP"]
        TxBuffer[3] = 15
        send(TxBuffer)
    
    #16
    def FeedBack_StatusWord(self):
        TxBuffer[3] = 16
        send(TxBuffer)

    #17
    def Set_target_position(self, Angle):
        gradi= self.Resolution/360*Angle
        TxBuffer[3] = 17
        TxBuffer[8] = gradi
        send(TxBuffer)

    #18
    def EncoderValue(self):
        TxBuffer[3] = 18
        send(TxBuffer)

    #19
    def VelocityValue(self):
        TxBuffer[3] = 19
        send(TxBuffer)
    
    

    