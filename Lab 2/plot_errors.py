import matplotlib.pyplot as plt
from utilities import FileReader




def plot_errors(filename):
    
    headers, values=FileReader(filename).read_file()
    
    time_list=[]
    
    first_stamp=values[0][-1]
    
    for val in values:
        time_list.append(val[-1] - first_stamp)

    
    plt.rcParams.update({'font.size': 18})
    fig, axes = plt.subplots(1,2, figsize=(14,6))

    try:
        axes[0].plot([lin[0] for lin in values], [lin[1] for lin in values], label="Path")
        axes[0].set_title("Path PID Control Parabola")
        axes[0].grid()
        axes[0].legend()
        axes[0].set(xlabel='X', ylabel='Y')

        
        axes[1].set_title("each individual state PID Control Parabola")
        for i in range(0, len(headers) - 1):
            axes[1].plot(time_list, [lin[i] for lin in values], label= headers[i])

        axes[1].legend()
        axes[1].grid()
        axes[1].set(xlabel='Time', ylabel='Magnitude of Value')
    except Exception as e:
        print(e)
    
    fig.tight_layout()
    plt.show()
    
    





import argparse

if __name__=="__main__":

    parser = argparse.ArgumentParser(description='Process some files.')
    parser.add_argument('--files', nargs='+', required=True, help='List of files to process')
    
    args = parser.parse_args()
    
    print("plotting the files", args.files)

    filenames=args.files
    for filename in filenames:
        plot_errors(filename)



