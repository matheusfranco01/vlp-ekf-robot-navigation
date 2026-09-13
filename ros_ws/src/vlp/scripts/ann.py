#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import rospy
import pickle
import torch
import torch.nn as nn
import rospkg

from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseWithCovarianceStamped
from custom_msg_python.msg import IlluminanceArray

package_root = rospkg.RosPack().get_path('vlp')
model_dir = os.path.join(package_root, 'scripts', 'models', 'model_RNA.pickle')

class RegressorMLP(nn.Module):
    def __init__(self, input_dim, hidden_dim, output_dim):
        super().__init__()
        
        self.hidden = nn.Sequential(
            nn.Linear(input_dim, hidden_dim),
            nn.ReLU(),
        )

        # Camada de saída
        self.output = nn.Linear(hidden_dim, output_dim)

    def forward(self, x):
        x = self.hidden(x)
        x = self.output(x)
        return x

rospy.init_node('ann_node')

with open(model_dir, 'rb') as f:
    model = pickle.load(f)

if isinstance(model, dict):
    loaded_model = RegressorMLP(input_dim=12, hidden_dim=128, output_dim=2)
    loaded_model.load_state_dict(model)
    model = loaded_model

model.eval()

ann_position_pub = rospy.Publisher('/ann_position', Point, queue_size=10)
ann_pose_pub = rospy.Publisher('/ann_pose', PoseWithCovarianceStamped, queue_size=10)

def ilu_callback(msg):
    iluminance_data = msg.iluminancias
    features = torch.tensor(iluminance_data, dtype=torch.float32)
    features = features.unsqueeze(0)
    with torch.no_grad():
        output = model(features)
    predicted_x = output[0][0].item() / 100.0
    predicted_y = output[0][1].item() / 100.0
    rospy.loginfo(f"Posição prevista: X={predicted_x:.2f}, Y={predicted_y:.2f}")
    ann_position_pub.publish(Point(predicted_x, predicted_y, 0.0))    

    pose_msg = PoseWithCovarianceStamped()
    pose_msg.header.stamp = msg.header.stamp
    pose_msg.header.frame_id = 'map'
    pose_msg.pose.pose.position.x = predicted_x
    pose_msg.pose.pose.position.y = predicted_y
    pose_msg.pose.pose.position.z = 0.3
    pose_msg.pose.pose.orientation.w = 1.0
    pose_msg.pose.covariance = [
        1.5, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 1.5, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 1e6,
    ]
    ann_pose_pub.publish(pose_msg)

# Leitura do sinal de iluminância
ilu_topic = rospy.get_param("~iluminance_topic", "/estimated_illuminance")
rospy.Subscriber(ilu_topic, IlluminanceArray, ilu_callback)
rospy.spin()