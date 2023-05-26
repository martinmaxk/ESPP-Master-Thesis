import csv
import numpy as np
import pandas as pd

# def read_csv(file_name):
#     res = []
#     with open(file_name) as csv_file:
#         csv_reader = csv.reader(csv_file, delimiter=',')
#         for row in csv_reader:
#             res.append(row)
    
#     return res

def filter_maps(vis_sub_name, non_vis_sub_name, map_type):
    vis_sub_file = pd.read_csv(vis_sub_name).to_numpy()

    non_vis_sub_file = pd.read_csv(non_vis_sub_name).to_numpy()

    map_names = vis_sub_file[:,0]

    new_file_idx = []

    i=0

    for row in non_vis_sub_file:
        if row[0] in map_names:
            new_file_idx.append(i)
        i=i+1

    # print(non_vis_sub_file[new_file_idx][2])

    with open(map_type + "VISSUBvs1XGRID.csv", 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerows(non_vis_sub_file[new_file_idx])
        print("wrote")

vis_sub_folder = "meshLambda-1Comp0"
filter_maps(vis_sub_folder + "/bgmaps.csv", "grid1/bgmaps.csv", "bgmaps")
filter_maps(vis_sub_folder + "/da2.csv", "grid1/da2.csv", "da2")
filter_maps(vis_sub_folder + "/dao.csv", "grid1/dao.csv", "dao")
filter_maps(vis_sub_folder + "/rooms.csv", "grid1/rooms.csv", "rooms")
