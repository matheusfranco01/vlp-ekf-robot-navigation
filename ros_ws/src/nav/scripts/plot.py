import rospy
import threading
import matplotlib.pyplot as plt
import message_filters
import json

from nav_msgs.msg import Odometry

ROOM_HALF_WIDTH = 5
ROOM_HALF_HEIGHT = 5
WALL_THICKNESS = 0.1

odom_trajectory_x = []
odom_trajectory_y = []
ground_truth_x = []
ground_truth_y = []
current_odom = (5, 5)
current_ground_truth = (5, 5)

# Listas para salvar os dados no formato JSON
odom_data_list = []
gt_data_list = []

odom_flag = False
ground_truth_flag = False

state_lock = threading.Lock()
odom_line = None
ground_truth_line = None
odom_point = None
g_t_point = None

def sync_callback(odom_msg, gt_msg):
    global current_odom, current_ground_truth, odom_flag, ground_truth_flag

    odom_x = odom_msg.pose.pose.position.x
    odom_y = odom_msg.pose.pose.position.y
    gt_x = gt_msg.pose.pose.position.x
    gt_y = gt_msg.pose.pose.position.y
    timestamp = odom_msg.header.stamp.to_sec()

    with state_lock:
        current_odom = (odom_x, odom_y)
        odom_trajectory_x.append(odom_x)
        odom_trajectory_y.append(odom_y)
        odom_data_list.append({"t": timestamp, "x": odom_x, "y": odom_y})

        current_ground_truth = (gt_x, gt_y)
        ground_truth_x.append(gt_x)
        ground_truth_y.append(gt_y)
        gt_data_list.append({"t": timestamp, "x": gt_x, "y": gt_y})

        odom_flag = True
        ground_truth_flag = True

def draw_room(ax):
    walls = [
        (0, -WALL_THICKNESS / 2.0, ROOM_HALF_WIDTH * 2.0, WALL_THICKNESS),
        (0, ROOM_HALF_HEIGHT * 2.0 - WALL_THICKNESS / 2.0, ROOM_HALF_WIDTH * 2.0, WALL_THICKNESS),
        (-WALL_THICKNESS / 2.0, 0, WALL_THICKNESS, ROOM_HALF_HEIGHT * 2.0),
        (ROOM_HALF_WIDTH * 2.0 - WALL_THICKNESS / 2.0, 0, WALL_THICKNESS, ROOM_HALF_HEIGHT * 2.0),
    ]
    for x, y, w, h in walls:
        ax.add_patch(plt.Rectangle((x, y), w, h, facecolor='lightgray', 
                                   edgecolor='black', alpha=0.8, zorder=0))

def update_plot(ax, fig):
    with state_lock:
        if not odom_flag and not ground_truth_flag:
            return 
        
        odom_xs = list(odom_trajectory_x)
        odom_ys = list(odom_trajectory_y)
        odom_x = current_odom[0]
        odom_y = current_odom[1]

        gt_xs = list(ground_truth_x)
        gt_ys = list(ground_truth_y)
        gt_x = current_ground_truth[0]
        gt_y = current_ground_truth[1]

        if odom_line is None or ground_truth_line is None:
            return

        odom_line.set_data(odom_xs, odom_ys)
        ground_truth_line.set_data(gt_xs, gt_ys)
        odom_point.set_data([odom_x], [odom_y])
        g_t_point.set_data([gt_x], [gt_y])

        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
        ax.grid(True)
        ax.set_aspect('equal', adjustable='box')
        ax.legend(loc='best')

        combined_xs = odom_xs + gt_xs
        combined_ys = odom_ys + gt_ys
        if len(combined_xs) >= 2:
            margem = 0.5
            ax.set_xlim(min(combined_xs) - margem, max(combined_xs) + margem)
            ax.set_ylim(min(combined_ys) - margem, max(combined_ys) + margem)

        fig.canvas.draw_idle()
        fig.canvas.flush_events()

rospy.init_node("odom_sync_listener", anonymous=False)

odom_topic = rospy.get_param("~odom_topic", "odometry/filtered")
ground_truth_topic = rospy.get_param("~ground_truth_topic", "/base_pose_ground_truth")

# Sincronizador de tópicos
sub_odom = message_filters.Subscriber(odom_topic, Odometry)
sub_gt = message_filters.Subscriber(ground_truth_topic, Odometry)

ts = message_filters.ApproximateTimeSynchronizer([sub_odom, sub_gt], queue_size=10, slop=0.05)
ts.registerCallback(sync_callback)

plt.ion()
fig, ax = plt.subplots(figsize=(8, 6))
draw_room(ax) 

odom_line, = ax.plot([], [], color='tab:blue')
ground_truth_line, = ax.plot([], [], color='tab:orange', linestyle='--')
odom_point, = ax.plot([], [], marker='o', color='tab:red', markersize=6, 
                      linestyle='None', label='Posição odom')
g_t_point, = ax.plot([], [], marker='o', color='tab:green', markersize=6, 
                     linestyle='None', label='Posição real')
ax.legend(loc='best')
ax.set_xlim(-0.5, ROOM_HALF_WIDTH * 2.0 + 0.5)
ax.set_ylim(-0.5, ROOM_HALF_HEIGHT * 2.0 + 0.5)

rospy.loginfo('Nó sincronizado iniciado. Aguardando dados...')

try:
    while not rospy.is_shutdown():
        update_plot(ax, fig)
        plt.pause(0.05)
except Exception:
    pass
finally:
    plt.ioff()
    plt.close('all')

    # Salva os dados em arquivos JSON ao encerrar
    with open('position.json', 'w') as f_odom:
        json.dump(odom_data_list, f_odom, indent=4)
    rospy.loginfo(f"[{len(odom_data_list)}] leituras salvas em position.json")

    with open('gt_gazebo.json', 'w') as f_gt:
        json.dump(gt_data_list, f_gt, indent=4)
    rospy.loginfo(f"[{len(gt_data_list)}] leituras salvas em gt_gazebo.json")