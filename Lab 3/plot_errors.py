
import matplotlib.pyplot as plt
from utilities import FileReader
import numpy as np



def plot_errors(filename):
    
    headers, values=FileReader(filename).read_file()
    print(np.array(values).shape)
    
    time_list=[]
    
    first_stamp=values[0][-1]
    
    for val in values:
        time_list.append(val[-1] - first_stamp)

    
    
    fig, axes = plt.subplots(2,1, figsize=(14,6))


    axes[0].plot([lin[len(headers) - 3] for lin in values], [lin[len(headers) - 2] for lin in values])
    axes[0].set_title("state space")
    axes[0].grid()

    
    axes[1].set_title("each individual state")
    for i in range(0, len(headers) - 1):
        axes[1].plot(time_list, [lin[i] for lin in values], label= headers[i])

    axes[1].legend()
    axes[1].grid()

    plt.show()
    
def plot_pose(filename):
    headers, values = FileReader(filename).read_file()
    first_stamp = [0][-1]
    
    values_np = np.array(values)

    imu_ax = values_np[:, 0]
    imu_ay = values_np[:, 1]
    kf_ax = values_np[:, 2]
    kf_ay = values_np[:, 3]
    kf_vx = values_np[:, 4]
    kf_w = values_np[:, 5]
    x = values_np[:, 6]
    y = values_np[:, 7]
    time = values_np[:, 8] - values_np[0, 8]
    time_list=[]
    
    first_stamp=values[0][-1]
    
    for val in values:
        time_list.append(val[-1] - first_stamp)
    fig, axes = plt.subplots(4,1, figsize=(10,10))
    for i in range(0, len(headers) - 1):
        axes[0].plot(time_list, [lin[i] for lin in values], label= headers[i])
    axes[0].set_title("All Signals")
    axes[0].grid()
    axes[0].legend()

    axes[1].plot(time, imu_ax, label="imu_ax")
    axes[1].plot(time, kf_ax, label="kf_ax")
    axes[1].set_title("X Acceleration")
    axes[1].grid()
    axes[1].legend()
    # axes[1].xlabel("Time")
    # axes[1].ylabel("Acceleration")

    axes[2].plot(time, imu_ay, label="imu_ay")
    axes[2].plot(time, kf_ay, label="kf_ay")
    axes[2].set_title("Y Acceleration")
    axes[2].grid()
    axes[2].legend()
    # axes[2].xlabel("Time")
    # axes[2].ylabel("Acceleration")

    axes[3].plot(time, kf_vx, label="kf_vx")
    axes[3].plot(time, kf_w, label="kf_w")
    axes[3].set_title("KF Velocity")
    axes[3].grid()
    axes[3].legend()
    # axes[3].xlabel("Time")
    # axes[3].ylabel("Velocity")
 
    fig.supxlabel("Time")
    fig.supylabel("Magnitude")
    fig.suptitle("Q covariance = 0.9")
    plt.tight_layout()
    plt.show()
    





import argparse

if __name__=="__main__":

    parser = argparse.ArgumentParser(description='Process some files.')
    parser.add_argument('--files', nargs='+', required=True, help='List of files to process')
    
    args = parser.parse_args()
    
    print("plotting the files", args.files)

    filenames=args.files
    for filename in filenames:
        #plot_errors(filename)
        plot_pose(filename)
