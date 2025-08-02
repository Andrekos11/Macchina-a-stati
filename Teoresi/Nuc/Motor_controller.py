#!/usr/bin/env python3

import rclpy.parameter
#from sirio_manipulator.AuxindLibrary import *
from sirio_manipulator.NUCLibrary import *
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32MultiArray
from manipulator_interfaces.msg import MotorCommand
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

VELOCITY_MODE = 1
POSITION_MODE = 0

class MotorController(Node):

    def __init__(self):
        
        super().__init__("arm_motor_controller_node")

        self.control_enabled = False

        self.get_logger().info("Inizializzazione del nodo arm_motor_controller_node")

        # Declare parameters
        self.declare_parameter("buffer_frequency", 200.0)    # up to 250 Hz, this is the frequency of the command buffer processing
        self.declare_parameter("feedback_frequency", 20.0)   # up to 30 Hz, this is the frequency of the feedback publishing
        self.declare_parameter("menu_cmd_frequency", 5.0)    # This is the frequency of the menu command processing
        self.declare_parameter("motors_id", [1, 2, 3, 4, 5, 6, 7])
        self.declare_parameter("command_mode", VELOCITY_MODE)
        self.declare_parameter("kp", 0.2) # Proportional gain for the velocity position mode

        self.declare_parameter("motors_resolution", [1] * 7)
        self.declare_parameter("motor_velocity_limits", [0.3] * 7) # Velocity limits for each motor in rad/s
        self.declare_parameter("motor_acceleration_limits", [0.05] * 7) # Acceleration limits for each motor in rad/s^2
        self.declare_parameter("homing_offsets", [0.0] * 7)

        self.motors_ids = self.get_parameter("motors_id").get_parameter_value().integer_array_value
        self.resolution = self.get_parameter("motors_resolution").get_parameter_value().integer_array_value
        self.command_mode = self.get_parameter("command_mode").get_parameter_value().integer_value
        self.kp_ = self.get_parameter("kp").get_parameter_value().double_value
        self.motor_velocity_limits = self.get_parameter("motor_velocity_limits").get_parameter_value().double_array_value
        self.motor_acceleration_limits = self.get_parameter("motor_acceleration_limits").get_parameter_value().double_array_value
        self.homing_offsets = self.get_parameter("homing_offsets").get_parameter_value().double_array_value
        # Convert homing offsets from degrees to radians
        self.homing_offsets = [math.radians(offset) for offset in self.homing_offsets]
        
        self.num_acceleration_samples = 2

        self.get_logger().info(f"Motor IDs: {self.motors_ids}")
        self.get_logger().info(f"Motor resolutions: {self.resolution}")

        if len(self.motors_ids) != len(self.resolution):
            self.get_logger().error("Il numero di ID motori e risoluzioni non corrisponde")
            raise ValueError("Il numero di ID motori e risoluzioni non corrisponde")
        
        self.command_buffer_ = []

        self.get_logger().info("Inizializzazione network CAN")

        self.network = InitNetwork()    

        self.get_logger().info("Inizializzazione della rete CAN")

        # Inizializza i messaggi con tanti zeri quanti sono i motori
        self.pos_cmd_ = [0] * len(self.motors_ids)
        self.vel_cmd_ = [0] * len(self.motors_ids)
        self.counter = 0
        self.encoder_value_ = [0 for k in range(len(self.motors_ids))]
        self.vel_status_ = [0 for k in range(len(self.motors_ids))]
        self.current_status_ = [0 for k in range(len(self.motors_ids))]
        self.accel_status = [0 for k in range(len(self.motors_ids))]

        self.nodi = []
        
        for index, id in enumerate(self.motors_ids):
            if id > 0:
                self.get_logger().info(f"Inizializzazione del motore con ID {id}")
                self.nodi.append(Auxind(id, self.network, self.resolution[index], self.homing_offsets[index]))

                if self.command_mode == POSITION_MODE:
                    self.nodi[index].SetPositionMode(self.motor_velocity_limits[index], self.motor_acceleration_limits[index])
                elif self.command_mode == VELOCITY_MODE:
                    self.nodi[index].SetVelocityMode()

                current_pos = self.nodi[index].EncoderValue()
                self.encoder_value_[index] = current_pos
                self.pos_cmd_[index] = current_pos
                self.nodi[index].SetEncoderToZero()
                time.sleep(0.5) #Wait for the motor to initialize

            else:
                self.get_logger().info(f"Motore con id {id} disabilitato")
                self.nodi.append(None)

        # Init pubs and subs
        self.init_pub_sub()

        self.past_vel_status_ = [0 for k in range(len(self.motors_ids))] # List to store past velocity status for acceleration calculation
        self.past_vel_status_timestamp_ = [Clock().now() for k in range(len(self.motors_ids))] # Timestamps for past velocity statuus


    def init_pub_sub(self):
        self.buffer_period = 1.0 / self.get_parameter("buffer_frequency").get_parameter_value().double_value
        self.feedback_period = 1.0 / self.get_parameter("feedback_frequency").get_parameter_value().double_value
        self.menu_cmd_period = 1.0 / self.get_parameter("menu_cmd_frequency").get_parameter_value().double_value

        self.get_logger().info(f"Buffer period: {self.buffer_period}")
        self.get_logger().info(f"Control period: {self.feedback_period}")
        self.get_logger().info(f"Menu command period: {self.menu_cmd_period}")

        # === Callback groups per ogni subscriber ===
        pos_sub_cb_group = MutuallyExclusiveCallbackGroup()
        vel_sub_cb_group = MutuallyExclusiveCallbackGroup()
        menu_sub_cb_group = MutuallyExclusiveCallbackGroup()

        # === Subscribers ===>
        self.pos_cmd_sub = self.create_subscription(
            Float32MultiArray,
            "sirio/arm/motors/pos_cmd",
            self.pos_cmd_callback,
            1,
            callback_group=pos_sub_cb_group
        )

        self.vel_cmd_sub = self.create_subscription(
            Float32MultiArray,
            "sirio/arm/motors/vel_cmd",
            self.vel_cmd_callback,
            1,
            callback_group=vel_sub_cb_group
        )

        self.menu_cmd_sub = self.create_subscription(
            MotorCommand,
            "sirio/arm/menu_cmd",
            self.menu_cmd_callback,
            1,
            callback_group=menu_sub_cb_group
        )

        # === Publishers ===
        self.pos_status_pub = self.create_publisher(
            Float32MultiArray,
            "sirio/arm/motors/pos_status",
            1
        )

        self.vel_status_pub = self.create_publisher(
            Float32MultiArray,
            "sirio/arm/motors/vel_status",
            1
        )

        self.current_status_pub = self.create_publisher(
            Float32MultiArray,
            "sirio/arm/motors/current_status",
            1
        )


    def publish_pos_status(self):

        self.control_enabled = True

        # self.feedback_counter += 1
        
        # for index, id in enumerate(self.nodi):
        #     if index !=0 and index !=6:
        #         self.encoder_value_[index] = self.nodi[index].EncoderValue(self.resolution[index])

        msg = Float32MultiArray(data=self.encoder_value_)
        self.pos_status_pub.publish(msg)


    def publish_vel_status(self):        
        msg = Float32MultiArray(data=self.vel_status_)
        self.vel_status_pub.publish(msg)

    # def publish_current_status(self):
    #     signed_currents = []

    #     if len(self.past_vel_status_) < self.num_acceleration_samples:
    #         return
        
    #     # Current must be positive or negative depending on the direction of the applied torque
    #     for i, current in enumerate(self.current_status_):
    #         #check if motor is accelerating
    #         d_vel = self.vel_status_[-1][i] - self.past_vel_status_[0][i]
    #         acc = d_vel / (self.num_acceleration_samples * self.feedback_period)
    #         if abs(acc) > self.min_acceleration:
    #             signed_currents.append(current if d_vel > 0 else -current)
    #             continue

    #         # If the motor is not accelerating, we check if it is maintaining a constant velocity
    #         if abs(self.vel_status_[i]) > 0.01:
    #             signed_currents.append(current if self.vel_status_[i] > 0 else -current)
    #             continue

    #         # Motor is stationary, current is considered positive
    #         signed_currents.append(current)

    #     msg = Float32MultiArray(data=signed_currents)
    #     self.current_status_pub.publish(msg)

    def spinner(self):

        control_status_cb_group = MutuallyExclusiveCallbackGroup()

        # === Timer: process_buffer ===
        self.timer = self.create_timer(
            self.buffer_period,
            self.control_loop,
            callback_group=control_status_cb_group
        )

        # === Timer: publish_pos_status and enable control loop ===
        self.timer_pub_pos_status = self.create_timer(
            self.feedback_period,
            self.publish_pos_status,
            callback_group=control_status_cb_group
        )

        self.timer_pub_vel_status = self.create_timer(
            self.feedback_period,
            self.publish_vel_status,
            callback_group=control_status_cb_group
        )

        # self.timer_pub_current_status = self.create_timer(
        #     self.feedback_period,
        #     self.publish_current_status,
        #     callback_group=control_status_cb_group
        # )

        # === Executor ===
        executor = MultiThreadedExecutor()
        executor.add_node(self)
        executor.spin()
    
    def pos_cmd_callback(self, msg: Float32MultiArray):
        for i, pos in enumerate(msg.data):
            self.pos_cmd_[i] = pos

    def vel_cmd_callback(self, msg: Float32MultiArray):
        for i, vel in enumerate(msg.data):
            self.vel_cmd_[i] = vel

    def heartbeat_callback(self):

        cmd_freq = self.cmd_counter / (self.get_clock().now().nanoseconds - self.cmd_time) * 1e9
        feedback_freq = self.feedback_counter / (self.get_clock().now().nanoseconds - self.feedback_time) * 1e9

        self.get_logger().info(f"Command frequency: {cmd_freq:.2f} Hz")
        self.get_logger().info(f"Feedback frequency: {feedback_freq:.2f} Hz")

        self.cmd_counter = 1
        self.feedback_counter = 1
        self.cmd_time = self.get_clock().now().nanoseconds
        self.feedback_time = self.get_clock().now().nanoseconds

    def menu_cmd_callback(self, msg: MotorCommand):
        if len(msg.ids) != len(msg.commands):
            self.get_logger().error("Il numero di ID motori e comandi non corrisponde")
            return
        
        for motor_id, command in zip(msg.ids, msg.commands):
            if motor_id not in self.motors_ids:
                self.get_logger().error(f"ID motore {motor_id} non valido")
                continue
            
            index = self.motors_ids.index(motor_id)
            node = self.nodi[index]

            if command == MotorCommand.COMMAND_SET_POSITION_MODE:
                self.command_mode = POSITION_MODE
                node.SetPositionMode(self.motor_velocity_limits[index], self.motor_acceleration_limits[index])
                self.get_logger().info(f"Modalità di comando impostata su POSIZIONE per il motore {motor_id}")
            elif command == MotorCommand.COMMAND_SET_VELOCITY_MODE:
                self.command_mode = VELOCITY_MODE
                node.SetVelocityMode()
                self.get_logger().info(f"Modalità di comando impostata su VELOCITÀ per il motore {motor_id}")
            elif command == MotorCommand.COMMAND_HOME_CW:
                self.pos_cmd_[self.nodi.index(node)] = self.homing_offsets[index]
                self.vel_cmd_[self.nodi.index(node)] = 0
                node.HomingModeCW()
                self.get_logger().info(f"Comando HOME_CW inviato al motore {motor_id}")
            elif command == MotorCommand.COMMAND_HOME_CCW:
                self.pos_cmd_[self.nodi.index(node)] = self.homing_offsets[index]
                self.vel_cmd_[self.nodi.index(node)] = 0
                node.HomingModeCCW()
                self.get_logger().info(f"Comando HOME_CCW inviato al motore {motor_id}")
            elif command == MotorCommand.COMMAND_SET_ZERO:
                self.pos_cmd_[self.nodi.index(node)] = 0
                self.vel_cmd_[self.nodi.index(node)] = 0
                node.SetEncoderToZero()
                self.get_logger().info(f"Comando SET_ZERO inviato al motore {motor_id}")
            else: 
                self.get_logger().error(f"Comando {command} non riconosciuto per il motore {motor_id}")

    def control_loop(self):

        if self.control_enabled:
            if self.nodi[self.counter] is not None:
                a=time.monotonic() 
                self.encoder_value_[self.counter] = self.nodi[self.counter].EncoderValue()
                b=time.monotonic()
                self.get_logger().info(f"Tempo di lettura dell'encoder per il motore {self.counter}: {b-a:.6f} secondi")
                # self.past_vel_status_[self.counter] = self.vel_status_[self.counter]
                # self.vel_status_[self.counter] = self.nodi[self.counter].VelocityValue()
                # d_vel = self.vel_status_[self.counter] - self.past_vel_status_[self.counter]
                # d_t = Clock().now() - self.past_vel_status_timestamp_[self.counter]
                # self.accel_status[self.counter] = d_vel / d_t.nanoseconds * 1e9
                # self.current_status_[self.counter] = self.nodi[self.counter].CurrentValue()

            if self.command_mode == POSITION_MODE and self.nodi[self.counter] is not None:
                self.nodi[self.counter].ProfilePositionAbsolute(self.pos_cmd_[self.counter])
            elif self.command_mode == VELOCITY_MODE and self.nodi[self.counter] is not None:
                # self.get_logger().info(f"Velocità motore {self.counter}: {self.vel_cmd_[self.counter]}")

                vel_cmd = self.vel_cmd_[self.counter] + self.kp_ * (self.pos_cmd_[self.counter] - self.encoder_value_[self.counter])

                #vel_cmd = min(max(vel_cmd, -self.motor_velocity_limits[self.counter]), self.motor_velocity_limits[self.counter])
                
                self.nodi[self.counter].ProfileVelocity(vel_cmd, 0.001, self.motor_acceleration_limits[self.counter])

        if self.counter==len(self.motors_ids) - 1: 
            self.counter = 0
            self.control_enabled = False
        else:
            self.counter += 1

if __name__ == "__main__":

    rclpy.init()
    motor_controller = MotorController()
    motor_controller.spinner()
    rclpy.shutdown()   

