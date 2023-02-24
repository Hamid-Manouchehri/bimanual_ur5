#!/usr/bin/env python3
"""
Author: Hamid Manouchehri
Year: 2022-2023

This module plot data saved in 'pathToCSVFile' directory.
'CSVFileName_livePlot_data' file will be created by the 'main_3D.py' or
'main_3D_linear_jac.py'.
"""
import os
from numpy import array
import csv
import rospy
import matplotlib.pyplot as plt
from sys import path
path.insert(1, '/home/rebel/ROS1_workspaces/bimanual_ur5_ws/src/ur5_description/config/')  # TODO: Insert dir of config folder of the package.
import config

titleOfPlot = "file_name"  # TODO
titleOfXaxis = "time (s)"  # TODO

# CSVFileName_livePlot_data = 'T_z_90_10_5_70_35.csv'  # TODO: manual change of file_name
CSVFileName_livePlot_data = config.bimanual_ur5_dic['CSVFileName']
pathToCSVFile = config.bimanual_ur5_dic['CSVFileDirectory']


def ReadCSV():
    """Read whole data from the CSV file."""
    csvData = []
    with open(pathToCSVFile + CSVFileName_livePlot_data, 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            csvData.append(row)
        return csvData



def main():
    """Compute main routine."""
    livePlotVal_x = []
    livePlotVal_y = []
    livePlotVal_y_des = []
    livePlotVal_y_actual = []

    if os.path.isfile(pathToCSVFile + CSVFileName_livePlot_data) is True:
        csvFileData = ReadCSV()  # string.

        titleOfYaxis = csvFileData[0][1]
        plotLegend = csvFileData[0][2:]

        numOfRows, numOfColumns = array(csvFileData[1:][:]).shape

        Data = array([[0.]*numOfColumns]*(numOfRows-1))

        for i in range(numOfRows-1):
            for j in range(numOfColumns):
                Data[i][j] = (float(csvFileData[i+1][j]))  # cast str to float
            livePlotVal_x.append(Data[i][0])
            livePlotVal_y.append(Data[i][1:])  # TODO  # TODO: uncomment for automatic call
            # livePlotVal_y_des.append(Data[i][1:4])  # TODO: for separated desired trajectory, notice to change boundary indeces for different data structures.
            # livePlotVal_y_actual.append(Data[i][4:])  # TODO: for separated actual trajectory, notice to change boundary indeces for different data structures.

        plt.figure(titleOfPlot)  # title of the figure
        plt.plot(livePlotVal_x, livePlotVal_y)  # TODO: uncomment for automatic call
        # plt.plot(livePlotVal_x, livePlotVal_y_des, linestyle='dotted')  # TODO: for separated desired trajectory
        # plt.plot(livePlotVal_x, livePlotVal_y_actual, linestyle='solid')  # TODO: for separated actual trajectory
        plt.xlabel(titleOfXaxis, fontsize=18)
        if csvFileData[0][0] == 'time':  # Check if header (as string) is added
            plt.ylabel(titleOfYaxis, fontsize=18)  # TODO: uncomment for automatic setting 'ylabel'
            # plt.ylabel('translational_traj object (m)', fontsize=18)  # TODO: for manual setting of 'ylabel'
            plt.legend(plotLegend, fontsize=18)
        plt.grid()
        plt.tight_layout()
        plt.tick_params(axis='x', labelsize=18)
        plt.tick_params(axis='y', labelsize=18)
        plt.show()

    else:
        print('There is no CSV source file in the defined directory!')
        print('Hint: You have may not called "WriteToCSV" method within the code!')



if __name__ == '__main__':

    try:
        main()

    except rospy.ROSInterruptException:
        pass
