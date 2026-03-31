#!/usr/bin/env python3

import threading
import time
from pathlib import Path
from functools import partial

import rclpy
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import Log
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from crazyflie_interfaces.msg import Status
import rowan

# Import compatibile con NiceGUI 1.2.24
from nicegui import Client, app, events, ui, Tailwind

# Variabile globale per accedere al nodo ROS dall'interfaccia
node = None

class NiceGuiNode(Node):

    def __init__(self) -> None:
        super().__init__('nicegui')

        # Inizializza dizionari vuoti per evitare crash prima che la UI sia pronta
        self.logs = dict()
        self.supervisor_labels = dict()
        self.battery_labels = dict()
        self.radio_labels = dict()
        self.robotmodels = dict()
        self.tabs_list = [] # Rinominato per evitare conflitti
        
        # --- LOGICA ROS (Eseguita nel Thread Secondario) ---
        
        # wait until the crazyflie_server is up and running
        self.emergencyService = self.create_client(Empty, 'all/emergency')
        self.get_logger().info('In attesa del servizio all/emergency...')
        # Nota: questo può bloccare un po', ma siamo in un thread separato quindi OK
        if not self.emergencyService.wait_for_service(timeout_sec=10.0):
             self.get_logger().error('Crazyflie server non trovato!')

        # find all crazyflies
        self.cfnames = []
        for srv_name, srv_types in self.get_service_names_and_types():
            if 'crazyflie_interfaces/srv/StartTrajectory' in srv_types:
                cfname = srv_name[1:-17]
                if cfname != 'all':
                    self.cfnames.append(cfname)
        
        self.get_logger().info(f'Trovati Crazyflies: {self.cfnames}')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 1)
        self.sub_log = self.create_subscription(Log, 'rosout', self.on_rosout, rclpy.qos.QoSProfile(
                        depth=1000,
                        durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL))

        for name in self.cfnames:
            self.create_subscription(Status, name + '/status', partial(self.on_status, name=name), 1)

        # Call on_timer function
        update_rate = 30 # Hz
        self.timer = self.create_timer(
            1.0/update_rate, 
            self.on_timer,
            clock=rclpy.clock.Clock(clock_type=rclpy.clock.ClockType.SYSTEM_TIME))
        
        self.normal_style = Tailwind().text_color('black').font_weight('normal')
        self.red_style = Tailwind().text_color('red-600').font_weight('bold')


    def setup_ui(self):
        # --- LOGICA GRAFICA (Eseguita nel Main Thread) ---
        
        with ui.row().classes('items-stretch'):
            with ui.card().classes('w-full h-full'):
                ui.label('Visualization').classes('text-2xl')
                with ui.scene(800, 400, on_click=self.on_vis_click) as scene:
                    for name in self.cfnames:
                        # Nota: path corretto per file statici
                        robot = scene.stl('/urdf/cf2_assembly.stl').scale(1.0).material('#ff0000').with_name(name)
                        self.robotmodels[name] = robot
                        
                        # augment with some additional fields
                        robot.status_ok = False
                        robot.battery_ok = False
                        robot.status_watchdog = time.time()
                        robot.supervisor_text = ""
                        robot.battery_text = ""
                        robot.radio_text = ""
                scene.camera.x = 0
                scene.camera.y = -1
                scene.camera.z = 2
                scene.camera.look_at_x = 0
                scene.camera.look_at_y = 0
                scene.camera.look_at_z = 0

        with ui.row().classes('w-full h-lvh'):
            with ui.tabs().classes('w-full') as tabs:
                self.tabs_list = []
                for name in ["all"] + self.cfnames:
                    self.tabs_list.append(ui.tab(name))
            
            with ui.tab_panels(tabs, value=self.tabs_list[0], on_change=self.on_tab_change).classes('w-full') as self.tabpanels:
                for name, tab in zip(["all"] + self.cfnames, self.tabs_list):
                    with ui.tab_panel(tab):
                        self.logs[name] = ui.log().classes('w-full h-96 no-wrap')
                        self.supervisor_labels[name] = ui.label("")
                        self.battery_labels[name] = ui.label("")
                        self.radio_labels[name] = ui.label("")

    def on_rosout(self, msg: Log) -> None:
        if msg.name == "crazyflie_server":
            if msg.msg.startswith("["):
                idx = msg.msg.find("]")
                if idx > 1:
                    name = msg.msg[1:idx]
                    if name == 'all':
                        for logname ,log in self.logs.items():
                            if logname != "all":
                                log.push(msg.msg)
                    elif name in self.logs:
                        self.logs[name].push(msg.msg[idx+2:])

        if 'all' in self.logs:
            self.logs['all'].push(msg.msg)

    def on_timer(self) -> None:
        # Se la UI non è ancora stata creata, esci
        if not self.robotmodels:
            return

        for name, robotmodel in self.robotmodels.items():
            ros_time = rclpy.time.Time() # get the latest
            robot_status_ok = robotmodel.status_ok and robotmodel.battery_ok
            robot_status_text = ""
            
            has_transform = False
            try:
                if self.tf_buffer.can_transform("world", name, ros_time):
                    t = self.tf_buffer.lookup_transform("world", name, ros_time)
                    has_transform = True
            except TransformException:
                pass

            if has_transform:
                transform_time = rclpy.time.Time.from_msg(t.header.stamp)
                transform_age = self.get_clock().now() - transform_time
                if transform_age.nanoseconds * 1e-9 > 1:
                    robot_status_ok = False
                    robot_status_text += "old transform; "
                else:
                    pos = t.transform.translation
                    robotmodel.move(pos.x, pos.y, pos.z)
                    robotmodel.rotate(*rowan.to_euler([
                        t.transform.rotation.w,
                        t.transform.rotation.x,
                        t.transform.rotation.y,
                        t.transform.rotation.z], "xyz"))
            else:
                robot_status_ok = False
                robot_status_text += "unavailable transform; "

            if time.time() - robotmodel.status_watchdog > 2.0:
                robot_status_ok = False
                robot_status_text += "no recent status update; "
                if name in self.supervisor_labels:
                    self.supervisor_labels[name].set_text(robot_status_text)
                    self.battery_labels[name].set_text("N.A.")
                    self.radio_labels[name].set_text("N.A.")
            else:
                if name in self.supervisor_labels:
                    self.supervisor_labels[name].set_text(robot_status_text + robotmodel.supervisor_text)
                    self.battery_labels[name].set_text(robotmodel.battery_text)
                    self.radio_labels[name].set_text(robotmodel.radio_text)

            if robot_status_ok:
                robotmodel.material('#00ff00')
            else:
                robotmodel.material('#ff0000')

            if name in self.battery_labels:
                if robotmodel.battery_ok:
                    self.normal_style.apply(self.battery_labels[name])
                else:
                    self.red_style.apply(self.battery_labels[name])

    def on_vis_click(self, e: events.SceneClickEventArguments):
        hit = e.hits[0]
        name = hit.object_name or hit.object_id
        ui.notify(f'You clicked on the {name}')
        if name == 'ground':
            self.tabpanels.value = 'all'
        else:
            self.tabpanels.value = name

    def on_status(self, msg, name) -> None:
        # Se la UI non è ancora pronta, ignoriamo il messaggio per evitare crash
        if name not in self.robotmodels:
            return

        status_ok = True
        is_flying = False
        supervisor_text = ""
        if msg.supervisor_info & Status.SUPERVISOR_INFO_CAN_BE_ARMED:
            supervisor_text += "can be armed; "
        if msg.supervisor_info & Status.SUPERVISOR_INFO_IS_ARMED:
            supervisor_text += "is armed; "
        if msg.supervisor_info & Status.SUPERVISOR_INFO_AUTO_ARM:
            supervisor_text += "auto-arm; "
        if msg.supervisor_info & Status.SUPERVISOR_INFO_CAN_FLY:
            supervisor_text += "can fly; "
        else:
            status_ok = False
        if msg.supervisor_info & Status.SUPERVISOR_INFO_IS_FLYING:
            supervisor_text += "is flying; "
            is_flying = True
        if msg.supervisor_info & Status.SUPERVISOR_INFO_IS_TUMBLED:
            supervisor_text += "is tumbled; "
            status_ok = False
        if msg.supervisor_info & Status.SUPERVISOR_INFO_IS_LOCKED:
            supervisor_text += "is locked; "
            status_ok = False
        self.robotmodels[name].supervisor_text = supervisor_text

        battery_text = f'{msg.battery_voltage:.2f} V'
        battery_ok = True
        
        if (is_flying and msg.battery_voltage < 3.2) or (not is_flying and msg.battery_voltage < 3.8):
            battery_ok = False
        if msg.pm_state == Status.PM_STATE_BATTERY:
            battery_text += " (on battery)"
        elif msg.pm_state == Status.PM_STATE_CHARGING:
            battery_text += " (charging)"
        elif msg.pm_state == Status.PM_STATE_CHARGED:
            battery_text += " (charged)"
        elif msg.pm_state == Status.PM_STATE_LOW_POWER:
            battery_text += " (low power)"
            battery_ok = False
        elif msg.pm_state == Status.PM_STATE_SHUTDOWN:
            battery_text += " (shutdown)"
            battery_ok = False
        self.robotmodels[name].battery_text = battery_text
        
        radio_text = f'{msg.rssi} dBm; Unicast: {msg.num_rx_unicast} / {msg.num_tx_unicast}; Broadcast: {msg.num_rx_broadcast} / {msg.num_tx_broadcast}; Latency: {msg.latency_unicast} ms'
        self.robotmodels[name].radio_text = radio_text

        self.robotmodels[name].status_ok = status_ok
        self.robotmodels[name].battery_ok = battery_ok
        self.robotmodels[name].status_watchdog = time.time()

    def on_tab_change(self, arg):
        for name, robotmodel in self.robotmodels.items():
            if name != arg.value:
                robotmodel.scale(1)
        if arg.value in self.robotmodels:
            self.robotmodels[arg.value].scale(2)


def ros_main() -> None:
    global node
    rclpy.init()
    # Qui creiamo SOLO la parte ROS, non la UI
    node = NiceGuiNode()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass

# Definizione della pagina principale
@ui.page('/')
def index():
    if node is None:
        ui.label('Inizializzazione ROS in corso... ricarica la pagina tra qualche secondo.')
        return
    
    # Qui, nel Main Thread di NiceGUI, costruiamo finalmente l'interfaccia
    node.setup_ui()

# Configurazione file statici (SENZA follow_symlink=True per compatibilità)
app.add_static_files("/urdf",
                     str((Path(__file__).parent.parent.parent / "share" / "crazyflie" / "urdf").resolve()))

# Avvio del thread ROS
app.on_startup(lambda: threading.Thread(target=ros_main).start())

ui.run.APP_IMPORT_STRING = f'{__name__}:app'
ui.run(uvicorn_reload_dirs=str(Path(__file__).parent.resolve()), favicon='🤖')



# #!/usr/bin/env python3

# import threading
# import time
# from pathlib import Path
# from functools import partial

# import rclpy
# from std_srvs.srv import Empty
# from geometry_msgs.msg import Twist
# from rcl_interfaces.msg import Log
# from rclpy.executors import ExternalShutdownException
# from rclpy.node import Node

# from tf2_ros import TransformException
# from tf2_ros.buffer import Buffer
# from tf2_ros.transform_listener import TransformListener

# from crazyflie_interfaces.msg import Status
# import rowan

# # Import compatibile con NiceGUI 1.2.24
# from nicegui import Client, app, events, ui, Tailwind

# # Variabile globale per accedere al nodo ROS dall'interfaccia
# node = None

# class NiceGuiNode(Node):

#     def __init__(self) -> None:
#         super().__init__('nicegui')

#         # Inizializza dizionari vuoti per evitare crash prima che la UI sia pronta
#         self.logs = dict()
#         self.supervisor_labels = dict()
#         self.battery_labels = dict()
#         self.radio_labels = dict()
#         self.robotmodels = dict()
#         self.tabs_list = [] # Rinominato per evitare conflitti
        
#         # --- LOGICA ROS (Eseguita nel Thread Secondario) ---
        
#         # wait until the crazyflie_server is up and running
#         self.emergencyService = self.create_client(Empty, 'all/emergency')
#         self.get_logger().info('In attesa del servizio all/emergency...')
#         # Nota: questo può bloccare un po', ma siamo in un thread separato quindi OK
#         if not self.emergencyService.wait_for_service(timeout_sec=10.0):
#              self.get_logger().error('Crazyflie server non trovato!')

#         # find all crazyflies
#         self.cfnames = []
#         for srv_name, srv_types in self.get_service_names_and_types():
#             if 'crazyflie_interfaces/srv/StartTrajectory' in srv_types:
#                 cfname = srv_name[1:-17]
#                 if cfname != 'all':
#                     self.cfnames.append(cfname)
        
#         self.get_logger().info(f'Trovati Crazyflies: {self.cfnames}')

#         self.tf_buffer = Buffer()
#         self.tf_listener = TransformListener(self.tf_buffer, self)

#         self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 1)
#         self.sub_log = self.create_subscription(Log, 'rosout', self.on_rosout, rclpy.qos.QoSProfile(
#                         depth=1000,
#                         durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL))

#         for name in self.cfnames:
#             self.create_subscription(Status, name + '/status', partial(self.on_status, name=name), 1)

#         # Call on_timer function
#         update_rate = 30 # Hz
#         self.timer = self.create_timer(
#             1.0/update_rate, 
#             self.on_timer,
#             clock=rclpy.clock.Clock(clock_type=rclpy.clock.ClockType.SYSTEM_TIME))
        
#         self.normal_style = Tailwind().text_color('black').font_weight('normal')
#         self.red_style = Tailwind().text_color('red-600').font_weight('bold')


#     def setup_ui(self):
#         # --- LOGICA GRAFICA (Eseguita nel Main Thread) ---
        
#         with ui.row().classes('items-stretch'):
#             with ui.card().classes('w-full h-full'):
#                 ui.label('Visualization').classes('text-2xl')
#                 with ui.scene(800, 400, on_click=self.on_vis_click) as scene:
#                     for name in self.cfnames:
#                         # Nota: path corretto per file statici
#                         robot = scene.stl('/urdf/cf2_assembly.stl').scale(1.0).material('#ff0000').with_name(name)
#                         self.robotmodels[name] = robot
                        
#                         # augment with some additional fields
#                         robot.status_ok = False
#                         robot.battery_ok = False
#                         robot.status_watchdog = time.time()
#                         robot.supervisor_text = ""
#                         robot.battery_text = ""
#                         robot.radio_text = ""
#                 scene.camera.x = 0
#                 scene.camera.y = -1
#                 scene.camera.z = 2
#                 scene.camera.look_at_x = 0
#                 scene.camera.look_at_y = 0
#                 scene.camera.look_at_z = 0

#         with ui.row().classes('w-full h-lvh'):
#             with ui.tabs().classes('w-full') as tabs:
#                 self.tabs_list = []
#                 for name in ["all"] + self.cfnames:
#                     self.tabs_list.append(ui.tab(name))
            
#             with ui.tab_panels(tabs, value=self.tabs_list[0], on_change=self.on_tab_change).classes('w-full') as self.tabpanels:
#                 for name, tab in zip(["all"] + self.cfnames, self.tabs_list):
#                     with ui.tab_panel(tab):
#                         self.logs[name] = ui.log().classes('w-full h-96 no-wrap')
#                         self.supervisor_labels[name] = ui.label("")
#                         self.battery_labels[name] = ui.label("")
#                         self.radio_labels[name] = ui.label("")

#     def on_rosout(self, msg: Log) -> None:
#         if msg.name == "crazyflie_server":
#             if msg.msg.startswith("["):
#                 idx = msg.msg.find("]")
#                 if idx > 1:
#                     name = msg.msg[1:idx]
#                     if name == 'all':
#                         for logname ,log in self.logs.items():
#                             if logname != "all":
#                                 log.push(msg.msg)
#                     elif name in self.logs:
#                         self.logs[name].push(msg.msg[idx+2:])

#         if 'all' in self.logs:
#             self.logs['all'].push(msg.msg)

#     def on_timer(self) -> None:
#         # Se la UI non è ancora stata creata, esci
#         if not self.robotmodels:
#             return

#         for name, robotmodel in self.robotmodels.items():
#             ros_time = rclpy.time.Time() # get the latest
#             robot_status_ok = robotmodel.status_ok and robotmodel.battery_ok
#             robot_status_text = ""
            
#             has_transform = False
#             try:
#                 if self.tf_buffer.can_transform("world", name, ros_time):
#                     t = self.tf_buffer.lookup_transform("world", name, ros_time)
#                     has_transform = True
#             except TransformException:
#                 pass

#             if has_transform:
#                 transform_time = rclpy.time.Time.from_msg(t.header.stamp)
#                 transform_age = self.get_clock().now() - transform_time
#                 if transform_age.nanoseconds * 1e-9 > 1:
#                     robot_status_ok = False
#                     robot_status_text += "old transform; "
#                 else:
#                     pos = t.transform.translation
#                     robotmodel.move(pos.x, pos.y, pos.z)
#                     robotmodel.rotate(*rowan.to_euler([
#                         t.transform.rotation.w,
#                         t.transform.rotation.x,
#                         t.transform.rotation.y,
#                         t.transform.rotation.z], "xyz"))
#             else:
#                 robot_status_ok = False
#                 robot_status_text += "unavailable transform; "

#             if time.time() - robotmodel.status_watchdog > 2.0:
#                 robot_status_ok = False
#                 robot_status_text += "no recent status update; "
#                 if name in self.supervisor_labels:
#                     self.supervisor_labels[name].set_text(robot_status_text)
#                     self.battery_labels[name].set_text("N.A.")
#                     self.radio_labels[name].set_text("N.A.")
#             else:
#                 if name in self.supervisor_labels:
#                     self.supervisor_labels[name].set_text(robot_status_text + robotmodel.supervisor_text)
#                     self.battery_labels[name].set_text(robotmodel.battery_text)
#                     self.radio_labels[name].set_text(robotmodel.radio_text)

#             if robot_status_ok:
#                 robotmodel.material('#00ff00')
#             else:
#                 robotmodel.material('#ff0000')

#             if name in self.battery_labels:
#                 if robotmodel.battery_ok:
#                     self.normal_style.apply(self.battery_labels[name])
#                 else:
#                     self.red_style.apply(self.battery_labels[name])

#     def on_vis_click(self, e: events.SceneClickEventArguments):
#         hit = e.hits[0]
#         name = hit.object_name or hit.object_id
#         ui.notify(f'You clicked on the {name}')
#         if name == 'ground':
#             self.tabpanels.value = 'all'
#         else:
#             self.tabpanels.value = name

#     def on_status(self, msg, name) -> None:
#         # Se la UI non è ancora pronta, ignoriamo il messaggio per evitare crash
#         if name not in self.robotmodels:
#             return

#         status_ok = True
#         is_flying = False
#         supervisor_text = ""
#         if msg.supervisor_info & Status.SUPERVISOR_INFO_CAN_BE_ARMED:
#             supervisor_text += "can be armed; "
#         if msg.supervisor_info & Status.SUPERVISOR_INFO_IS_ARMED:
#             supervisor_text += "is armed; "
#         if msg.supervisor_info & Status.SUPERVISOR_INFO_AUTO_ARM:
#             supervisor_text += "auto-arm; "
#         if msg.supervisor_info & Status.SUPERVISOR_INFO_CAN_FLY:
#             supervisor_text += "can fly; "
#         else:
#             status_ok = False
#         if msg.supervisor_info & Status.SUPERVISOR_INFO_IS_FLYING:
#             supervisor_text += "is flying; "
#             is_flying = True
#         if msg.supervisor_info & Status.SUPERVISOR_INFO_IS_TUMBLED:
#             supervisor_text += "is tumbled; "
#             status_ok = False
#         if msg.supervisor_info & Status.SUPERVISOR_INFO_IS_LOCKED:
#             supervisor_text += "is locked; "
#             status_ok = False
#         self.robotmodels[name].supervisor_text = supervisor_text

#         battery_text = f'{msg.battery_voltage:.2f} V'
#         battery_ok = True
        
#         if (is_flying and msg.battery_voltage < 3.2) or (not is_flying and msg.battery_voltage < 3.8):
#             battery_ok = False
#         if msg.pm_state == Status.PM_STATE_BATTERY:
#             battery_text += " (on battery)"
#         elif msg.pm_state == Status.PM_STATE_CHARGING:
#             battery_text += " (charging)"
#         elif msg.pm_state == Status.PM_STATE_CHARGED:
#             battery_text += " (charged)"
#         elif msg.pm_state == Status.PM_STATE_LOW_POWER:
#             battery_text += " (low power)"
#             battery_ok = False
#         elif msg.pm_state == Status.PM_STATE_SHUTDOWN:
#             battery_text += " (shutdown)"
#             battery_ok = False
#         self.robotmodels[name].battery_text = battery_text
        
#         radio_text = f'{msg.rssi} dBm; Unicast: {msg.num_rx_unicast} / {msg.num_tx_unicast}; Broadcast: {msg.num_rx_broadcast} / {msg.num_tx_broadcast}; Latency: {msg.latency_unicast} ms'
#         self.robotmodels[name].radio_text = radio_text

#         self.robotmodels[name].status_ok = status_ok
#         self.robotmodels[name].battery_ok = battery_ok
#         self.robotmodels[name].status_watchdog = time.time()

#     def on_tab_change(self, arg):
#         for name, robotmodel in self.robotmodels.items():
#             if name != arg.value:
#                 robotmodel.scale(1)
#         if arg.value in self.robotmodels:
#             self.robotmodels[arg.value].scale(2)


# def ros_main() -> None:
#     global node
#     rclpy.init()
#     # Qui creiamo SOLO la parte ROS, non la UI
#     node = NiceGuiNode()
#     try:
#         rclpy.spin(node)
#     except ExternalShutdownException:
#         pass

# # Definizione della pagina principale
# @ui.page('/')
# def index():
#     if node is None:
#         ui.label('Inizializzazione ROS in corso... ricarica la pagina tra qualche secondo.')
#         return
    
#     # Qui, nel Main Thread di NiceGUI, costruiamo finalmente l'interfaccia
#     node.setup_ui()

# # Configurazione file statici (SENZA follow_symlink=True per compatibilità)
# app.add_static_files("/urdf",
#                      str((Path(__file__).parent.parent.parent / "share" / "crazyflie" / "urdf").resolve()))

# # Avvio del thread ROS
# app.on_startup(lambda: threading.Thread(target=ros_main).start())

# ui.run.APP_IMPORT_STRING = f'{__name__}:app'
# ui.run(uvicorn_reload_dirs=str(Path(__file__).parent.resolve()), favicon='🤖')