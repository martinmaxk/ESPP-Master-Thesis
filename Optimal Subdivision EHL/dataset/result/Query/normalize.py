import os
import csv
from collections import defaultdict

def read_csv(file_path):
    data = defaultdict(list)
    with open(file_path, 'r') as file:
        reader = csv.DictReader(file)
        for row in reader:
            row['ViaLabelsWithPrunedAvg'] = float(row['ViaLabelsCheckedAvg']) + float(row['ViaLabelsPrunedAvg'])
            map_key = row.pop('Map')
            if map_key not in data:
                data[map_key].append(row)
            else:
                print(file_path + " " + map_key)
    return data

def write_csv(name, file_path, data):
    is_empty = (not os.path.exists(file_path)) or os.stat(file_path).st_size == 0
    with open(file_path, 'a', newline='') as file:
        writer = csv.writer(file)
        if is_empty:
            header = list(data[0].keys())
            header.remove('Map')
            header = ['Map'] + header
            writer.writerow(header)
        if name and (not is_empty):
            writer.writerow([])
        if name:
            writer.writerow([name])
        for row in data:
            map_type = row['Map']
            row.pop('Map')
            row_to_write = [map_type] + list(row.values())
            writer.writerow(row_to_write)

def divide_columns(a, b):
    divided_row = {}
    for row_a, row_b in zip(a, b):
        for key in row_a.keys():
            if key == 'PointLocationTime':
                divided_row[key] = float(row_a[key]) / float(row_a['TotalTime'])
            elif key in row_b and key != 'Map' and key != 'NumVertices' and key != 'M-CdtRegions' and float(row_b[key]) != 0:
                divided_row[key] = float(row_a[key]) / float(row_b[key])
            else:
                divided_row[key] = float(row_a[key])
    return divided_row

def calculate_mean(divided):
    mean_values = {}
    num_rows = len(divided)
    for row in divided:
        for key, value in row.items():
            if key in mean_values:
                mean_values[key] += value
            else:
                mean_values[key] = value
    for key in mean_values.keys():
        mean_values[key] /= num_rows
    return mean_values

def iterate_matching_files(folder_a, folder_b, folder_c):
    files_a = set(os.listdir(folder_a))
    files_b = set(os.listdir(folder_b))
    common_files = sorted(files_a.intersection(files_b))
    
    for file_name in common_files:
        path_a = os.path.join(folder_a, file_name)
        path_b = os.path.join(folder_b, file_name)
        path_c = os.path.join(folder_c, file_name)
        yield (path_a, path_b, path_c)

def write_csv_for_folders(folder_a, folder_b, folder_c, file_path_out, normalize = True):
    if normalize:
        file_path_out += '_norm';
    file_path_out += '.csv'
    i = 0
    # Iterate over matching file paths
    for file_path_a, file_path_b, file_path_c in iterate_matching_files(folder_a, folder_b, folder_c):
        # Read CSV files
        data_a = read_csv(file_path_a)
        data_b = read_csv(file_path_b)
        data_c = read_csv(file_path_c)

        # Divide columns
        divided_data = []
        for map_key in data_a.keys():
            if map_key not in data_c:
                continue
            if normalize:
                divided_row = divide_columns(data_a[map_key], data_b[map_key])
            else:
                divided_row = {}
                for row_a in data_a[map_key]:
                    for key in row_a.keys():
                        divided_row[key] = float(row_a[key])
            divided_data.append(divided_row)
        mean_values = calculate_mean(divided_data)
        mean_values['Map'] = os.path.basename(file_path_a)
        
        name = None
        if i == 0:
            name = os.path.basename(os.path.dirname(file_path_a))
        mean_values['MapsIncludedRatio'] = str(len(divided_data)) + "/" + str(maptype_map_count[mean_values['Map']])
        # Write to CSV file
        write_csv(name, file_path_out, [mean_values])
        i += 1

maptype_map_count = {}
def main(normalize):
    for map_type in os.listdir('grid1'):
        grid_path = os.path.join('grid1', map_type)
        grid_data = read_csv(grid_path)
        maptype_map_count[map_type] = len(grid_data)
        
    folder_b = 'grid1'
    file_path_c = 'lambda_agg'
    for lam in [1, 5, 9, -1]:
        folder_a = 'meshLambda' + str(lam) + 'Comp0'
        write_csv_for_folders(folder_a, folder_b, folder_a, file_path_c, normalize)

    file_path_c = 'compr_agg'
    for lam in [9, -1]:
        folder_b = 'meshLambda' + str(lam) + 'Comp0'
        for compr in [1, 3, 5, 6, 10]:
            folder_a = 'meshLambda' + str(lam) + 'Comp' + str(compr)
            write_csv_for_folders(folder_a, folder_b, folder_a, file_path_c, normalize)

    folder_b = 'grid1'
    file_path_c = 'grid_agg'
    for grid_res in [1, 4, 16, 64]:
        folder_a = 'grid' + str(grid_res)
        write_csv_for_folders(folder_a, folder_b, folder_a, file_path_c, normalize)


    folder_b = 'grid1'
    folder_c = 'meshLambda-1Comp0'
    file_path_c = 'lambda_agg_vissub'
    for lam in [1, 5, 9]:
        folder_a = 'meshLambda' + str(lam) + 'Comp0'
        write_csv_for_folders(folder_a, folder_b, folder_c, file_path_c, normalize)

    folder_b = 'grid1'
    file_path_c = 'grid_agg_vissub'
    for grid_res in [1, 4, 16, 64]:
        folder_a = 'grid' + str(grid_res)
        write_csv_for_folders(folder_a, folder_b, folder_c, file_path_c, normalize)

main(True)
main(False)