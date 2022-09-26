import csv

CSV_DATA = '/home/hirayama-d/tsukuchare22_ws/src/potential_planner/data/waypoint_csv/sample_waypoint.csv'
def main():
    waypoint_csv = [list(map(float,line.rstrip().split(","))) for line in open(CSV_DATA).readlines()]
    print(waypoint_csv,type(waypoint_csv))
    # waypoint = [[0]*2]*len(waypoint_csv)
    # print(waypoint,type(waypoint))
    for i in range(len(waypoint_csv)):
        print("x , y",waypoint_csv[i][0], waypoint_csv[i][1])
if __name__ == "__main__":
    main()