import canopen
import can
from ComunicationLib import*
import time
import math
import threading
M_PI = math.pi


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

class Network:
    def __init__(self):
        self.network = canopen.Network() 
        self.network.connect(channel='can0', interface='socketcan', bitrate=1000000)
        send(self.network)
        print("inizializzo")
        self.Bus = can.interface.Bus(channel='can0', bustype='socketcan')
        #return self.network     

    def GetNetwork(self):
        return self.network

    def EncoderValue_Request(self, Nodo):
        self.Bus.send(can.Message(arbitration_id=0x80, data=[], is_extended_id=False))
        self.EncoderValue_Response(Nodo)

    def EncoderValue_Response(self, Nodo):
        max_wait = 0.010
        start_time = time.time()
        while time.time() - start_time < max_wait:
            i=0
            for i in range(8):
                message = self.Bus.recv(timeout=0.01)
                if message is not None:
                    Id = message.arbitration_id - 0x280
                    Nodo[Id].setEncoderValue(((struct.unpack('<i', message.data[0:4])[0]) * (2 * M_PI)) / 1000000)
        return 0.0

class Auxind:

    def __init__(self, NodeId, network, resolution, Offset):   
        #self.Nodo.nmt.state = 'PRE-OPERATIONAL'
        #self.Bus = bus
        self.Nodo = network.add_node(NodeId, 'auxindPic.eds')
        self.stato = 0
        self.Resolution = resolution
        self.offset = Offset
        self.Encoder = 0
        self.COB_ID_encoder = 0x280 + NodeId
        self.COB_ID_velocity = 0x380 + NodeId
        self.COB_ID_status = 0x180 + NodeId

        #NMT
        self.NMT_saved = 0
        self.NMT_request = 127

        #State
        self.State_HighLevel = -1
        self.State_PowerDisable = -1
        self.State_Fault = -1
        self.State_PowerEnable = -1
        self.State_request = 0

        #Event
        self.Event_shutdwon = False
        self.Event_SwitchOn = False
        self.Event_DisableVoltage = False
        self.Event_QuickStop = False
        self.Event_DisableOperation = False
        self.Event_EnableOperation = False
        self.Event_FaultReset = False

        #Variabili
        self.Speed = 0
        self.Acceleration = 0
        self.Angolo = 0
        self.funzione = 0
        self.PastMode = 0
        self.Mode = 0      
        self.Nodo.nmt.state = 'OPERATIONAL'
        time.sleep(0.5)
        self.Nodo.nmt.state = 'OPERATIONAL'
        if self.Nodo is not None:
            for x in range(5):
                send(15)
                time.sleep(0.1)
        #     return None


    #1    
    def SetVelocityMode(self):
        self.Nodo.sdo[0x6040].bits[0]= ControlWord["SHUT_DOWN"]
        time.sleep(0.2)
        self.Nodo.sdo[0x6060].raw = ModesOfOperation["PROFILE_VELOCITY"]
        time.sleep(0.2)
        self.Nodo.sdo[0x60FF].raw = 0
        self.Nodo.sdo[0x607F].raw = 300000
        self.Nodo.sdo[0x6083].raw = 1000
        self.Nodo.sdo[0x6084].raw = 1000
        time.sleep(0.2)
        self.Nodo.sdo[0x6040].bits[0]= ControlWord["SWITCH_ON"]
        time.sleep(0.2)
        self.Nodo.sdo[0x6040].bits[0]= ControlWord["ENABLE_OPERATION"]
        time.sleep(0.2)

    #2
    def SetPositionMode(self, Speed, acceleration):
        speed = abs((self.Resolution*Speed)/(2*M_PI))
        acc = abs((acceleration * self.Resolution) / (2 * M_PI))
        self.Nodo.sdo[0x6060].raw = ModesOfOperation["PROFILE_POSITION"] 
        #parametri del motore
        self.Nodo.sdo[0x6081].raw =speed
        self.Nodo.sdo[0x6083].raw = acc
        self.Nodo.sdo[0x6084].raw = acc 
        # Shutdown
        self.Nodo.sdo[0x6040].raw = ControlWord["SHUT_DOWN"]
        time.sleep(0.2)
        # Switch on
        self.Nodo.sdo[0x6040].raw = ControlWord["SWITCH_ON"]
        time.sleep(0.2)

    #3
    def HomingModeCW(self):
        self.Nodo.sdo[0x6060].raw = ModesOfOperation["HOMING"]
        self.Nodo.sdo.download(0x6098, 0, b'\x12')        #11 antiorario     12 orario
        self.Nodo.sdo[0x6099][1].raw = 15000  # homing speed research
        self.Nodo.sdo[0x6099][2].raw = 15000  # homing speed release
        self.Nodo.sdo[0x609A].raw = 100  # homing acceleration
        self.Nodo.sdo[0x607C].raw = self.offset # offset
        # Shutdown
        self.Nodo.sdo[0x6040].raw = ControlWord["SHUT_DOWN"]
        time.sleep(0.2)

        # Switch on
        self.Nodo.sdo[0x6040].raw = ControlWord["SWITCH_ON"]
        time.sleep(0.2)

        # Attiva il movimento
        self.Nodo.sdo[0x6040].raw = ControlWord["START_HOMING"]  #start homing
        time.sleep(0.2)


    #4
    def HomingModeCCW(self):
        self.Nodo.sdo[0x6060].raw = ModesOfOperation["HOMING"]
        self.Nodo.sdo.download(0x6098, 0, b'\x11')        #11 antiorario     12 orario
        self.Nodo.sdo[0x6099][1].raw = 15000  # homing speed research
        self.Nodo.sdo[0x6099][2].raw = 15000  # homing speed release
        self.Nodo.sdo[0x609A].raw = 100  # homing acceleration
        self.Nodo.sdo[0x607C].raw = self.offset # offset

        # Shutdown
        self.Nodo.sdo[0x6040].raw = ControlWord["SHUT_DOWN"]
        time.sleep(0.2)

        # Switch on
        self.Nodo.sdo[0x6040].raw = ControlWord["SWITCH_ON"]
        time.sleep(0.2)

        # Attiva il movimento
        self.Nodo.sdo[0x6040].raw = ControlWord["START_HOMING"]  #start homing
        time.sleep(0.2)

    #5
    def SetEncoderToZero (self):
        self.Nodo.sdo[0x6060].raw = ModesOfOperation["HOMING"]
        self.Nodo.sdo.download(0x6098, 0, b'\x23')        #11 antiorario     12 orario
        self.Nodo.sdo[0x6099][1].raw = 15000  # homing speed research
        self.Nodo.sdo[0x6099][2].raw = 15000  # homing speed release
        self.Nodo.sdo[0x609A].raw = 100  # homing acceleration
        self.Nodo.sdo[0x607C].raw = self.offset # offset

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
    def ProfileVelocity(self, Speed, Acc, MaxVel):
        speed = (self.Resolution*Speed)/(2*M_PI)
        acc = (self.Resolution*Acc)/(2*M_PI)
        maxvel = (self.Resolution*MaxVel)/(2*M_PI)
        self.Nodo.sdo[0x60FF].raw= speed
        self.Nodo.sdo[0x6083].raw = acc
        self.Nodo.sdo[0x6084].raw = acc
        self.Nodo.sdo[0x607F].raw = maxvel

    def ProfileVelocityRPDO(self, Speed):
        self.Nodo.rpdo[2]['Control word'].phys = ControlWord["ENABLE_OPERATION"]
        self.Nodo.rpdo[2]['Target velocity'].phys = (self.Resolution*Speed)/(2*M_PI)
        self.Nodo.rpdo[2].transmit()

    #7
    def ProfilePositionRelative(self, Angle):

        self.Nodo.sdo[0x607A].raw = int((self.Resolution*Angle)/(2*M_PI))
        
        if self.stato==0:
            try:
                self.Nodo.sdo[0x607A].raw = int((self.Resolution*Angle)/(2*M_PI))
            except canopen.sdo.exceptions.SdoCommunicationError:
                print("SdoCommunicationError")
            self.stato = 1
        else:
            self.Nodo.sdo[0x6040].raw = ControlWord["CLEAR_BIT"] #clear bit 4

        self.Nodo.sdo[0x6040].raw = ControlWord["ENABLE_POS_RELATIVE"]

    #8
    def ProfilePositionAbsolute(self, Angle):      
        self.Nodo.sdo[0x607A].raw = int((self.Resolution*Angle)/(2*M_PI))    
        
        if self.stato==0:
            try:
                self.Nodo.sdo[0x607A].raw = (self.Resolution*Angle)/(2*M_PI) 
            except canopen.sdo.exceptions.SdoCommunicationError:
                print("SdoCommunicationError")
            self.stato = 1
        else:
            self.Nodo.sdo[0x6040].raw = ControlWord["CLEAR_BIT"] #clear bit 4
    
        self.Nodo.sdo[0x6040].raw = ControlWord["ENABLE_POS_ABSOLUTE"]

    #9
    def Shutdown(self):
        self.Nodo.sdo[0x6040].bits[0]= ControlWord["SHUT_DOWN"]

    #10
    def Disable_Voltage(self):
        self.Nodo.sdo[0x6040].bits[0]= ControlWord["DISABLE_VOLTAGE"]

    #11
    def Switch_On(self):
        self.Nodo.sdo[0x6040].bits[0]= ControlWord["SWITCH_ON"]

    #12
    def Disable_Operation(self):
        self.Nodo.sdo[0x6040].bits[0]= ControlWord["DISABLE_OPERATION"]

    #13
    def Enable_Operation(self):
        self.Nodo.sdo[0x6040].bits[0]= ControlWord["ENABLE_OPERATION"]

    #14
    def Fault_Reset(self):
        self.Nodo.sdo[0x6040].bits[0]= ControlWord["FAULT_RESET"]

    #15
    def Quick_Stop(self):
        self.Nodo.sdo[0x6040].bits[0]= ControlWord["QUICK_STOP"]

    #16
    def FeedBack_StatusWord(self):
        # message = self.Bus.recv()
        # if message.arbitration_id == self.COB_ID_status:
        #     return struct.unpack('<h', message.data[0:2])[0]
        # else:
        #     return 0
        StatusWord=self.Nodo.sdo[0x6041].raw
        return StatusWord

    #17
    def Set_target_position(self, Angle):
        gradi= self.Resolution/360*Angle
        self.Nodo.sdo[0x607A].raw = gradi

    
    #18
    # def EncoderValue_Request(self):
    #     self.Bus.send(can.Message(arbitration_id=0x80, data=[], is_extended_id=False))

    def setEncoderValue(self, value):
        self.Encoder = value

    def EncoderValue(self):
        return self.Encoder
    #     max_wait=0.001
    #     start_time = time.time()
    #     while time.time() - start_time < max_wait:
    #         message = self.Bus.recv(timeout=0.001)
    #         if message is not None and message.arbitration_id == self.COB_ID_encoder:
    #             self.Encoder = ((struct.unpack('<i', message.data[0:4])[0])*(2*M_PI))/self.Resolution
    #     return 0.0
        
    #19
    def VelocityValue(self):
        # message = self.Bus.recv()
        # if message.arbitration_id == self.COB_ID_velocity:
        #     return ((struct.unpack('<i', message.data[0:4])[0])*(2*M_PI))/self.Resolution
        # else:
        #     return 0
        CurrentVelocity=self.Nodo.sdo[0x606C].raw
        CurrentVelocity=(CurrentVelocity*(2*M_PI))/self.Resolution
        #CurrentVelocity= CurrentVelocity*360/self.Resolution
        return CurrentVelocity

    
    

    # def setFunzione(self,Fun, Mod, Vel, Acc, Ang):
    #     self.funzione = Fun
    #     self.Speed = Vel
    #     self.Acceleration = Acc
    #     self.Angolo = Ang
    #     self.Mode = Mod
    
    def setFunction(self, Fun):
        self.funzione = Fun
    def setSpeed(self, Vel):
        self.funzione = Vel
    def setMode(self, Mod):
        self.funzione = Mod
    def setAcceleration(self, Acc):
        self.funzione = Acc
    def setAngle(self, Rad):
        self.funzione = Rad
    
        

    def NMT_SetRequest(self, request):
        self.NMT_request=request
    
    def State_SetRequest(self, request):
        self.State_request=request

    def Event_SetRequest(self, request):
        if request == 0x06:
            self.Event_shutdwon = True
            self.Event_SwitchOn = False
            self.Event_DisableVoltage = False
            self.Event_QuickStop = False
            self.Event_DisableOperation = False
            self.Event_EnableOperation = False
            self.Event_FaultReset = False
        elif request == 0x07:
            self.Event_shutdwon = False
            self.Event_SwitchOn = True
            self.Event_DisableVoltage = False
            self.Event_QuickStop = False
            self.Event_DisableOperation = True
            self.Event_EnableOperation = False
            self.Event_FaultReset = False
        elif request == 0x0F:
            self.Event_shutdwon = False
            self.Event_SwitchOn = False
            self.Event_DisableVoltage = False
            self.Event_QuickStop = False
            self.Event_DisableOperation = False
            self.Event_EnableOperation = True
            self.Event_FaultReset = False
        elif request == 0x00:
            self.Event_shutdwon = False
            self.Event_SwitchOn = False
            self.Event_DisableVoltage = True
            self.Event_QuickStop = False
            self.Event_DisableOperation = False
            self.Event_EnableOperation = False
            self.Event_FaultReset = False
        elif request == 0x02:
            self.Event_shutdwon = False
            self.Event_SwitchOn = False
            self.Event_DisableVoltage = False
            self.Event_QuickStop = True
            self.Event_DisableOperation = False
            self.Event_EnableOperation = False
            self.Event_FaultReset = False
        elif request == 0x08:
            self.Event_shutdwon = False
            self.Event_SwitchOn = False
            self.Event_DisableVoltage = False
            self.Event_QuickStop = False
            self.Event_DisableOperation = False
            self.Event_EnableOperation = False
            self.Event_FaultReset = True

    def NMT_ReadRequest(self):
        return self.NMT_request
    def NMT_ReadPastRequest(self):
        return self.NMT_saved
    
    def State_ReadRequest(self):
        return self.State_request
    



