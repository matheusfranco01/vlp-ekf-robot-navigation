#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import sys

import rospy
import rospkg
import numpy as np

script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)

from custom_msg_python.msg import IlluminanceArray
from nav_msgs.msg import Odometry

from estimator.utils import run_ies2rad
from estimator.ambient import generate_room
from estimator.estimator import dynamic_illuminance_window
from estimator.fft import FFT_filter

package_root = rospkg.RosPack().get_path('vlp')

# Diretórios
dir_ies = os.path.join(package_root, 'scripts', 'estimator', 'IES_files')
output_dir = os.path.join(package_root, 'data', 'test_01/')

# Nomes de arquivos
IES = "LEDtube"
filename = 'DadosTempo'
filename_a = "parameters"

# Extensões de arquivos
ext_a = ".ies"
ext_b = ".json"
ext_c = ".txt"

scale_factor_x = 10
scale_factor_y = 10
scale_factor_z = 5

floor_level = 0.3
SAMPLE_PERIOD = 10e-6
SAMPLE_WINDOW_DURATION = 10e-3
FFT_WINDOW_SIZE = int(round(SAMPLE_WINDOW_DURATION / SAMPLE_PERIOD))

simulation_start_stamp = None

os.makedirs(output_dir, exist_ok=True)
for folder in ["ambient", "objects", "materials", "output"]:
    os.makedirs(os.path.join(output_dir, folder), exist_ok=True)

wall_material = {
    'type': 'plastic',
    'color_rgb': (.02, .04, .065),
    'specularity': 0.00,
    'roughness': 0.00
}

# Configurações do ambiente e luminárias
ambient_sets = {
        'ambient': {
            'room_sizes': {'x': scale_factor_x * 1, 'y': scale_factor_y * 1, 'z': scale_factor_z * 1},
            'floor_level': floor_level,
            'sample_frequency': 100e3,
            'wall_bounces': 0,
            'walls': [
                {'x': 0},
                {'x': scale_factor_x * 1},
                {'y': 0},
                {'y': scale_factor_y * 1}
            ]
        },
        'luminaries': {
            "positions": [
                {
                    "x": 2.0,
                    "y": 2.5,
                    "z": 4.8
                },
                {
                    "x": 4.0,
                    "y": 2.5,
                    "z": 4.8
                },
                {
                    "x": 6.0,
                    "y": 2.5,
                    "z": 4.8
                },
                {
                    "x": 8.0,
                    "y": 2.5,
                    "z": 4.8
                },
                {
                    "x": 2.0,
                    "y": 5.0,
                    "z": 4.8
                },
                {
                    "x": 4.0,
                    "y": 5.0,
                    "z": 4.8
                },
                {
                    "x": 6.0,
                    "y": 5.0,
                    "z": 4.8
                },
                {
                    "x": 8.0,
                    "y": 5.0,
                    "z": 4.8
                },
                {
                    "x": 2.0,
                    "y": 7.5,
                    "z": 4.8
                },
                {
                    "x": 4.0,
                    "y": 7.5,
                    "z": 4.8
                },
                {
                    "x": 6.0,
                    "y": 7.5,
                    "z": 4.8
                },
                {
                    "x": 8.0,
                    "y": 7.5,
                    "z": 4.8
                }
            ],
            'ies_file_path': os.path.join(dir_ies, IES + ext_a),
            "modulation_frequencies": [4750, 8500, 12250, 16000, 19850, 23600, 27350, 31100, 34850, 38600, 42350, 46250]
        },
    }

run_ies2rad(
    ambient_sets['luminaries']['ies_file_path'],
    os.path.join(output_dir, 'objects', 'luminaire')
)

generate_room(
    ambient_sets['ambient']['room_sizes'],
    wall_material['type'],
    wall_material['color_rgb'],
    wall_material['specularity'],
    wall_material['roughness'],
    output_dir
)

def illuminance_estimator(msg):
    global simulation_start_stamp

    # Extrair os dados do ground truth e calcular uma amostra no tempo atual
    point = {
        'x': msg.pose.pose.position.x,
        'y': msg.pose.pose.position.y,
        'z': 0.30
    }

    msg_stamp = msg.header.stamp if msg.header.stamp.to_sec() > 0.0 else rospy.Time.now()
    if simulation_start_stamp is None:
        simulation_start_stamp = msg_stamp

    current_time = (msg_stamp - simulation_start_stamp).to_sec()

    illuminance_window = dynamic_illuminance_window(
        output_dir,
        ambient_sets['luminaries'],
        ambient_sets['ambient']['wall_bounces'],
        current_time,
        point,
        ambient_sets['ambient']['sample_frequency'],
        SAMPLE_WINDOW_DURATION,
    )

    # Aplicar filtro FFT para obter a iluminância modulada a partir da janela de 10 ms
    ilu_array = FFT_filter(
        np.asarray(illuminance_window, dtype=float),
        ambient_sets['ambient']['sample_frequency'],
        ambient_sets['luminaries']['modulation_frequencies']
    )

    # Caracterizando a mensagem a ser enviada
    illuminance_msg = IlluminanceArray()
    illuminance_msg.header.stamp = msg_stamp
    illuminance_msg.iluminancias = ilu_array.tolist() if hasattr(ilu_array, 'tolist') else ilu_array
    estimated_illuminance_pub.publish(illuminance_msg)


rospy.init_node('estimator_node')
estimated_illuminance_pub = rospy.Publisher('/estimated_illuminance', IlluminanceArray, queue_size=10)

gt_topic = rospy.get_param("~ground_truth_topic", "/base_pose_ground_truth")
rospy.Subscriber(gt_topic, Odometry, illuminance_estimator, queue_size=1, buff_size=2**24)

rospy.spin()

