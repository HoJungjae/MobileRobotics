from count_cubes import *
import numpy as np
from glob import glob

datapath = 'data/'

ground_truth = np.genfromtxt(datapath + 'ground_truth.csv', delimiter=',', dtype='int')
answer = dict([(item[0],(item[1], item[2])) for item in ground_truth])

score = 0

imgs = [f for f in glob(datapath+'*') if '.jpg' in f]

for f in imgs:
    i = int(f[-6:-4])
    img = cv2.imread(f,-1)

    num_yellow, num_green = count_cubes(img)

    if num_yellow == answer[i][0] and num_green == answer[i][1]:
        score += 1
        print(f)
    else:
        print(f"{f} not good. Expected: {answer[i][0]} {answer[i][1]}")

# Uncomment hsv_tool(Insert image) to use the hsv slider tool
#hsv_tool("data/img01.jpg")

print("Score: {}/{}".format(score, len(imgs)))