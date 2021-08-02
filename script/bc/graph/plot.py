import matplotlib.pyplot as plt
import csv
import numpy as np
import argparse
parser = argparse.ArgumentParser(description='arguments')
parser.add_argument('--name', type=str, default='temp', help='csv file name')
#parser.add_argument('--test', '-t', action='calculate RMSE', help='RMSE values')
args = parser.parse_args()

def save_graph_to_image(name, ylim=0.5):
    with open('graph/'+name+'.csv') as file:
        csv_data = []
        for line in file.readlines():
            csv_data.append(line.split(','))

    graph = csv_data[0:-5]
    graph = np.array(graph)
    graph = np.array(graph[:,0:3], dtype='float32')
    plt.title(name+': train & test loss')
    plt.plot(graph[:,0],graph[:,1], label='train_loss')
    plt.plot(graph[:,0],graph[:,2], label='test_loss')
    plt.legend()
    plt.ylim(0,ylim)
    plt.savefig('graph/'+name+'.png')
    plt.close()

if __name__ == '__main__':
    with open(args.name+'.csv') as file:
        csv_data = []
        for line in file.readlines():
            csv_data.append(line.split(','))

    graph = csv_data[0:-5]
    print(graph)
    graph = np.array(graph)
    graph = np.array(graph[:,0:3], dtype='float32')
    plt.title(args.name+': train & test loss')
    plt.plot(graph[:,0],graph[:,1], label='train_loss')
    plt.plot(graph[:,0],graph[:,2], label='test_loss')
    plt.legend()
    plt.savefig(args.name+'.png')
    plt.show()
    plt.close()

