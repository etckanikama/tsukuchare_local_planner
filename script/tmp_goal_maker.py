import csv
import numpy as np


def main():
    csv_float=[list(map(float,line.rstrip().split(","))) for line in open('/home/sakata-r/tsukuchare22_ws/src/gps_sub/bag/2/global_test_datafile.csv').readlines()]
    save = 0
    j = 0
    waypoint = [[0]*2]*len(csv_float)
    f = open('/home/sakata-r/tsukuchare22_ws/src/gps_sub/bag/2/waypoint.csv', 'w')
    writer = csv.writer(f)
    # print(len(csv_float))
    #読み込んだGNSSの値からwaypointを1.5m間隔で配置
    for i in range(0, len(csv_float)):
        #print(csv_float[i][0], csv_float[i][1])
        pre = np.array((csv_float[save][0], csv_float[save][1]))
        now = np.array((csv_float[i][0], csv_float[i][1]))
        distance = np.linalg.norm(pre-now)
        if distance > 1.5:
            waypoint[j][0] = csv_float[i][0]
            waypoint[j][1] = csv_float[i][1]
            print(waypoint[j][0], waypoint[j][1])
            waypoint_list = [waypoint[j][0], waypoint[j][1]]
            save = i
            writer.writerow(waypoint_list)
    f.close()

    
if __name__ == "__main__":
    main()
