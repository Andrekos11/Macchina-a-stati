import canopen
from LibreriaAuxind import *


class Node:

    def __init__(self):

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

    def setFunzione(self,Fun, Mod, Vel, Acc, Ang):
        self.funzione = Fun
        self.Speed = Vel
        self.Acceleration = Acc
        self.Angolo = Ang
        self.Mode = Mod


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
    



