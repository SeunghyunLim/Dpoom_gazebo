
import csv
import numpy as np
import os
import random
import math
import torch
import torch.nn as nn
from torch.utils.data import Dataset
from torch.utils.data import DataLoader
import torch.optim as optim
import argparse
import requests
from time import sleep
#import wandb


HISTORY = 10
EARLY_STOPPING = 100

seed = 42
torch.manual_seed(seed)
np.random.seed(seed)
random.seed(seed)

parser = argparse.ArgumentParser(description='arguments')
parser.add_argument('--ratio', type=float, default=0.7,
                    help='ratio for splitting dataset')
parser.add_argument('--lr', type=float, default=0.00001,
                    help='learning rate')
parser.add_argument('--min_lr', type=float, default=0.0001,
                    help='learning rate')
parser.add_argument('--max_lr', type=float, default=0.0001,
                    help='learning rate')
parser.add_argument('--batch', type=int, default=32, help='batch_size')
parser.add_argument('--name', type=str, default='temp',
                    help='checkpoint file name')
parser.add_argument('--epoch', type=int, default=1000, help='number of epoch')
parser.add_argument('--workers', type=int, default=1,
                    help='number of parallel data load workers')
parser.add_argument('--resume', '-r', action='store_true',
                    help='resume from checkpoint')
parser.add_argument('--validation', action='store_true', help='valiation only')
# parser.add_argument('--test', '-t', action='calculate RMSE', help='RMSE values')
args = parser.parse_args()

#wandb.init()
#wandb.config.update(args)

resume = False
validation = False


def csv2list(filename):
    raw_data = []
    with open(filename, newline='') as csvfile:
        spamreader = csv.reader(csvfile, delimiter=',', quotechar='|')

        # 10 Hz dynamics
        for i, row in enumerate(spamreader):
            if i % 2 == 0:
                raw_data.append(row)

    return raw_data[1:]

def make_history_data(split_data):
    # [vx1, vx2, vx3, vx4, vy1 .... d_vx4]
    total_history_data = []
    total_answer = []
    data = np.array(split_data, dtype='float32')
    for i in range(HISTORY-1, len(data)-HISTORY):  # last data only used for answer
        # history_data = []
        # if i < HISTORY-1:
        #     for j in range(i):
        #         history_data.append(data[0])
        #     for j in range(HISTORY-i):
        #         history_data.append(data[j+1])
        # else:
        history_data = data[i-HISTORY+1:i+1]

        history_data = np.array(history_data)
        goal_x = data[i][1]
        goal_y = data[i][2]

        deadends = history_data[:, 5:] / 360.0

        commands = history_data[:, 3:5]

        dead_ends = np.hstack(deadends)
        commands = np.hstack(commands)

        split_history_data = list(dead_ends)
        split_history_data.extend(list(commands))
        split_history_data.extend([goal_x, goal_y])
        #print(split_history_data)
        answer = data[i+1][[3, 4]]  # multiple indexing

        total_history_data.append(split_history_data)
        total_answer.append(list(answer))

    return total_history_data, total_answer

    # print(history_data)


train_dataX = []
train_dataY = []
filename = 'data/dynamic5_static4_center.csv'
split_data = csv2list(filename)
x, y = make_history_data(split_data)
train_dataX.extend(x)
train_dataY.extend(y)


# shuffle train data for split test data
temp = list(zip(train_dataX, train_dataY))
random.shuffle(temp)
train_dataX, train_dataY = zip(*temp)


# test will be shuffled in data loader
len_data = len(train_dataX)
test_dataX = train_dataX[int(len_data*0.7):]
test_dataY = train_dataY[int(len_data*0.7):]
train_dataX = train_dataX[0:int(len_data*0.7)]
train_dataY = train_dataY[0:int(len_data*0.7)]


val_dataX = []
val_dataY = []
filename = 'data/dynamic5_static4_center.csv'
split_data = csv2list(filename)
x, y = make_history_data(split_data)
val_dataX.extend(x)
val_dataY.extend(y)


print("train data x,y: {}, {}".format(len(train_dataX), len(train_dataY)))
print("test data x,y: {}, {}".format(len(test_dataX), len(test_dataY)))
print("val data x,y: {}, {}".format(len(val_dataX), len(val_dataY)))


class CustomDataset(Dataset):
    def __init__(self, x, y):
        self.x_data = x
        self.y_data = y

    def __len__(self):
        return len(self.x_data)

    def __getitem__(self, item):
        x_ = torch.FloatTensor(self.x_data[item])
        y_ = torch.FloatTensor(self.y_data[item])
        return x_, y_


# trainX, trainY = norm_data(train_dataX, train_dataY)
# testX, testY = norm_data(test_dataX, test_dataY)

train_dataset = CustomDataset(train_dataX, train_dataY)
train_loader = DataLoader(train_dataset, batch_size=args.batch, shuffle=True,
                          num_workers=args.workers, pin_memory=True)  # shuffle every epoch
test_dataset = CustomDataset(test_dataX, test_dataY)
test_loader = DataLoader(test_dataset, batch_size=args.batch,
                         shuffle=False, num_workers=args.workers, pin_memory=True)
val_dataset = CustomDataset(val_dataX, val_dataY)
val_loader = DataLoader(val_dataset, batch_size=1, shuffle=False)

# device = 'cuda' if torch.cuda.is_available() else 'cpu'
device = 'cpu'
best_error = 100
start_epoch = 0


class SimpleNet(torch.nn.Module):

    def __init__(self, in_features, out_features, hidden_features):
        super(SimpleNet, self).__init__()
        self.lin1 = nn.Linear(in_features=in_features,
                              out_features=hidden_features, bias=True)
        self.lin2 = nn.Linear(in_features=hidden_features,
                              out_features=hidden_features * 2, bias=True)
        self.lin3 = nn.Linear(in_features=hidden_features * 2,
                              out_features=hidden_features * 3, bias=True)
        self.lin4 = nn.Linear(in_features=hidden_features * 3,
                              out_features=hidden_features, bias=True)
        self.lin5 = nn.Linear(in_features=hidden_features,
                              out_features=out_features, bias=True)
        # self.lin4 = nn.Linear(in_features=hidden_features * 3,
        #                       out_features=hidden_features * 6, bias=True)
        # self.lin5 = nn.Linear(in_features=hidden_features * 6,
        #                       out_features=hidden_features * 3, bias=True)
        # self.lin6 = nn.Linear(in_features=hidden_features * 3,
        #                       out_features=out_features, bias=True)
        self.act = nn.ReLU()

    def forward(self, x):
        prev_x = x.clone().detach()  # save previous state
        x = self.act(self.lin1(x))
        x = self.act(self.lin2(x))
        x = self.act(self.lin3(x))
        x = self.act(self.lin4(x))
        x = self.lin5(x)
        # x = self.act(self.lin5(x))
        # x = self.lin6(x)
        # return x
        return x

n_deadends = 7
n_goal = 2
n_command = 2

input_size = HISTORY*(n_deadends+n_command)+n_goal
model = SimpleNet(input_size, n_command, input_size)
model = model.to(device)  # kaiming init
#wandb.watch(model)

if device == 'cuda':
    model = nn.DataParallel(model)
    torch.backends.cudnn.benchmark = True
print('Am I using CPU or GPU : {}'.format(device))
if resume:
    print('==> Resuming from checkpoint')
    assert os.path.isdir('checkpoint'), 'Error: no checkpoint dir found'
    checkpoint = torch.load('./checkpoint/' + args.name + '.pth')
    model.load_state_dict(checkpoint['model'])
    best_error = checkpoint['error']
    start_epoch = checkpoint['epoch']

optimizer = torch.optim.Adam(model.parameters(), lr=args.min_lr)
# optimizer = torch.optim.SGD(model.parameters(), lr=args.lr)
# optimizer = torch.optim.AdamW(model.parameters(), lr=args.lr)
# optimizer = AdamP(model.parameters(), lr=args.lr)t
criterion = nn.MSELoss()


def train(epoch, train_loader):
    print('\nEpoch: %d' % epoch)
    # print('Training {} batches, {} data'.format(len(train_loader), len(train_loader)*args.batch))
    model.train()

    train_loss = torch.FloatTensor([0])
    for batch_idx, samples in enumerate(train_loader):
        x_train, y_train = samples
        x_train, y_train = x_train.to(device), y_train.to(device)
        prediction = model(x_train)
        loss = criterion(prediction, y_train)
        optimizer.zero_grad(set_to_none=True)  # efficient zero out
        loss.backward()
        optimizer.step()

        train_loss += loss

    # scheduler.step()
    err = math.sqrt(float(train_loss.item() / float(len(train_loader))))
    print('Training ==> Epoch {:2d} / {} Cost: {:.6f}'.format(epoch, args.epoch,
                                                              err))
    return err


best_test_rmse_vx = 0
best_test_rmse_vy = 0
best_test_rmse_yawrate = 0
best_test_max_vx = 0
best_test_max_vy = 0
best_test_max_yawrate = 0


def test(epoch, val=True):
    global best_error, best_test_rmse_vx, best_test_rmse_vy, best_test_rmse_yawrate, best_test_max_vx, best_test_max_vy, best_test_max_yawrate
    model.eval()
    test_loss = torch.FloatTensor([0])
    val_loss = torch.FloatTensor([0])

    rmse_vx = torch.FloatTensor([0])  # tensor  with 1 dim, 0.0 element
    rmse_vy = torch.FloatTensor([0])  # tensor  with 1 dim, 0.0 element
    rmse_yawrate = torch.FloatTensor([0])  # tensor  with 1 dim, 0.0 element
    max_vx_err = torch.FloatTensor(0)  # empty tensor 0 dim
    max_vy_err = torch.FloatTensor(0)  # empty tensor 0 dim
    max_yawrate_err = torch.FloatTensor(0)  # empty tensor 0 dim
    bool_best = False

    if not val:
        with torch.no_grad():
            for batch_idx, samples in enumerate(test_loader):
                x_test, y_test = samples
                x_test, y_test = x_test.to(device), y_test.to(device)
                prediction = model(x_test)
                test_loss += criterion(prediction, y_test)
                rmse_vx += criterion(prediction[:, 0], y_test[:, 0])
                rmse_vy += criterion(prediction[:, 1], y_test[:, 1])
                # view(1): tensor(0) -> tensor([0])
                max_vx_err = torch.cat([max_vx_err, torch.max(
                    torch.abs(prediction[:, 0] - y_test[:, 0])).view(1)])
                max_vy_err = torch.cat([max_vy_err, torch.max(
                    torch.abs(prediction[:, 1] - y_test[:, 1])).view(1)])

        err = math.sqrt(float(test_loss.item() / float(len(test_loader))))
        print(
            'Testing ==> Epoch {:2d} / {} Cost: {:.6f}'.format(epoch, args.epoch, err))

        rmse_vx = math.sqrt(float(rmse_vx.item() / float(len(test_loader))))
        rmse_vy = math.sqrt(float(rmse_vy.item() / float(len(test_loader))))
        max_vx_err = torch.max(max_vx_err).item()
        max_vy_err = torch.max(max_vy_err).item()
        #max_yawrate_err = torch.max(max_yawrate_err).item()
        print('vx RMSE : {}'.format(rmse_vx))
        print('vy RMSE : {}'.format(rmse_vy))
        print('vx Max Err : {}'.format(max_vx_err))
        print('vy Max Err : {}'.format(max_vy_err))

        '''
        wandb.log({"Test Loss": err,
                   "vx RMSE": rmse_vx,
                   "vy RMSE": rmse_vy,
                   "yawrate RMSE": rmse_yawrate,
                   "vx Max Err": max_vx_err,
                   "vy Max Err": max_vy_err,
                   "yawrate Max Err":  max_yawrate_err})
        '''

        if err < best_error:
            print('Saving...')
            state = {
                'model': model.state_dict(),
                'error': err,
                'epoch': epoch,
            }
            if not os.path.isdir('checkpoint'):
                os.mkdir('checkpoint')
            torch.save(state, './checkpoint/' + args.name + '.pth')
            best_error = err
            bool_best = True

            # save best test err to send notify to my lord
            best_test_rmse_vx = rmse_vx
            best_test_rmse_vy = rmse_vy
            #best_test_rmse_yawrate = rmse_yawrate
            best_test_max_vx = max_vx_err
            best_test_max_vy = max_vy_err
            #best_test_max_yawrate = max_yawrate_err
        return err, bool_best, rmse_vx, rmse_vy, max_vx_err, max_vy_err
    else:
        with torch.no_grad():
            for batch_idx, samples in enumerate(val_loader):
                x_test, y_test = samples
                x_test, y_test = x_test.to(device), y_test.to(device)
                prediction = model(x_test)
                rmse_vx += criterion(prediction[:, 0], y_test[:, 0])
                rmse_vy += criterion(prediction[:, 1], y_test[:, 1])
                #rmse_yawrate += criterion(prediction[:, 2], y_test[:, 2])
                # view(1): tensor(0) -> tensor([0])
                max_vx_err = torch.cat([max_vx_err, torch.max(
                    torch.abs(prediction[:, 0] - y_test[:, 0])).view(1)])
                max_vy_err = torch.cat([max_vy_err, torch.max(
                    torch.abs(prediction[:, 1] - y_test[:, 1])).view(1)])

        print('Validation')
        rmse_vx = math.sqrt(float(rmse_vx.item() / float(len(val_loader))))
        rmse_vy = math.sqrt(float(rmse_vy.item() / float(len(val_loader))))
        max_vx_err = torch.max(max_vx_err).item()
        max_vy_err = torch.max(max_vy_err).item()
        #max_yawrate_err = torch.max(max_yawrate_err).item()
        print('vx RMSE : {}'.format(rmse_vx))
        print('vy RMSE : {}'.format(rmse_vy))
        #print('yaw rate RMSE : {}'.format(rmse_yawrate))
        print('vx Max Err : {}'.format(max_vx_err))
        print('vy Max Err : {}'.format(max_vy_err))
        #print('yaw rate Max Err : {}'.format(max_yawrate_err))
        return rmse_vx, rmse_vy, max_vx_err, max_vy_err


if not validation:
    f = open('graph/'+args.name+'.csv', 'w', encoding='utf-8', newline='')
    wr = csv.writer(f)

    early_stopping = 0
    for epoch in range(start_epoch, start_epoch + args.epoch):
        train_loss = train(epoch, train_loader)
        test_loss, bool_best, rmse_vx, rmse_vy, max_vx_err, max_vy_err = test(
            epoch, val=False)
        wr.writerow([epoch, train_loss, test_loss, bool_best,
                     rmse_vx, rmse_vy, max_vx_err, max_vy_err])

        early_stopping += 1
        if bool_best:
            early_stopping = 0
        if early_stopping > EARLY_STOPPING:
            break
        # scheduler.step()

print('Finish Training ==> Resuming from best checkpoint')
assert os.path.isdir('checkpoint'), 'Error: no checkpoint dir found'
checkpoint = torch.load('./checkpoint/' + args.name + '.pth')
model.load_state_dict(checkpoint['model'])

rmse_vx, rmse_vy, max_vx_err, max_vy_err = test(
    0, val=True)
if not validation:
    wr.writerow(["epoch", "train_loss", "test_loss", "bool_best",
                 "rmse_vx", "rmse_vy", "max_vx_err", "max_vy_err"])
    wr.writerow(["best: test_rmse_vx", "rmse_vy",
                 "max_vx_err", "max_vy_err"])
    wr.writerow([best_test_rmse_vx, best_test_rmse_vy,
                 best_test_max_vx, best_test_max_vy])
    wr.writerow(["validation: rmse_vx", "rmse_vy",
                 "max_vx_err", "max_vy_err"])
    wr.writerow([rmse_vx, rmse_vy,
                 max_vx_err, max_vy_err])
    f.close()


broadcast_msg = 'Finish training at {} epoch. \nName: {}. \nFor Best Test: \
                \nRMSE vx: {:.6f} \nRMSE vy: {:.6f} \
                \nMax vx error: {:.6f} \nMax vy error:  {:.6f}   \
                \n\nFor Validation: \nRMSE vx: {:.6f} \nRMSE vy: {:.6f}\
                 \nMax vx error: {:.6f} \nMax vy error: {:.6f} '.format(
                epoch, args.name, best_test_rmse_vx, best_test_rmse_vy, 
                best_test_max_vx, best_test_max_vy, rmse_vx, rmse_vy,  max_vx_err,
    max_vy_err)
'''
response = requests.post(TARGET_URL, headers={'Authorization': 'Bearer ' + TOKEN},
                            data={'message': broadcast_msg})
'''

TARGET_URL = 'https://notify-api.line.me/api/notify'
TOKEN = 'o3NVij6VHZd2Q3ULHXUiGYcgQqBrQot0T5414J6s2Ql'

if not validation:
    from graph import plot
    plot.save_graph_to_image(args.name, ylim=0.1)
    with open('graph/'+args.name+'.png', 'rb') as file:
        # 요청합니다.
        response = requests.post(
            TARGET_URL,
            headers={
                'Authorization': 'Bearer ' + TOKEN
            },
            data={
                'message': broadcast_msg,
            },
            files={
                'imageFile': file
            }
        )
    print("Notifying to your iphone complete, sir")
