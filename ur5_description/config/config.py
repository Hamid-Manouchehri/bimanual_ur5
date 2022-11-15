#!/usr/bin/env python3
"""
This file is created to configure some of static variables of executable scripts.
"""
bimanual_ur5_dic = {'CSVFileDirectory' : '/home/rebel/ROS1_workspaces/bimanual_ur5_ws/src/ur5_gazebo/scripts/',
                    'CSVFileName' : 'plot_data.csv',  # define name of the csv file.
                    'urdfDirectory' : '/home/rebel/ROS1_workspaces/bimanual_ur5_ws/src/ur5_description/urdf/',
                    'urdfModelName_r' : 'ur5_right_joint_limited_robot.urdf',
                    'urdfModelName_l' : 'ur5_left_joint_limited_robot.urdf'}

plot_csv_dic = {'CSVFileDirectory' : bimanual_ur5_dic['CSVFileDirectory'],
                'CSVFileName' : bimanual_ur5_dic['CSVFileName']}
