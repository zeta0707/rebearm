import argparse
import torch
from torch.utils.data import DataLoader
from iknet import IKDataset, IKNet

LOC_IN_Y = 1
LOC_OUT_Y = 3
MAX_Y = 3

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--kinematics-pose-csv", type=str, default="./dataset/test/kinematics_pose.csv"
    )

    parser.add_argument("--batch-size", type=int, default=10000)
    args = parser.parse_args()

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    #y-> data[1],2,3
    model = IKNet(MAX_Y)
    model.load_state_dict(torch.load("iknet_y.pth"))
    model.to(device)
    model.eval()

    dataset = IKDataset(args.kinematics_pose_csv, LOC_IN_Y, LOC_OUT_Y, MAX_Y)
    test_loader = DataLoader(dataset, batch_size=args.batch_size, shuffle=False)

    total_loss = 0.0
    for data, target in test_loader:
        data, target = data.to(device), target.to(device)
        output = model(data)
        total_loss += (output - target).norm().item() / args.batch_size
    print(f"Total loss = {total_loss}")

if __name__ == "__main__":
    main()
