import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

results_path = '/home/x/Documents/landing_pipeline/yolo_dataset/runs/detect/train/results.csv'

results = pd.read_csv(results_path)

#         train/box_loss,         train/cls_loss,         train/dfl_loss
#           val/box_loss,           val/cls_loss,           val/dfl_loss
#       metrics/mAP50(B)
#    metrics/mAP50-95(B)
print("metrics/mAP50(B)")
min_index = results['       metrics/mAP50(B)'].argmax()
min_epoch = results['                  epoch'][min_index]
last_epoch = results['                  epoch'].array[-1]
print(min_epoch)
print(results['       metrics/mAP50(B)'][min_index])
print(last_epoch - min_epoch)

print("metrics/mAP50-95(B)")
min_index = results['    metrics/mAP50-95(B)'].argmax()
min_epoch = results['                  epoch'][min_index]
last_epoch = results['                  epoch'].array[-1]
print(min_epoch)
print(results['    metrics/mAP50-95(B)'][min_index])
print(last_epoch - min_epoch)


plt.figure()
plt.plot(results['                  epoch'], results['         train/cls_loss'], label='train loss')
plt.plot(results['                  epoch'], results['           val/cls_loss'], label='val loss', c='red')
plt.grid()
plt.title('Loss vs epochs')
plt.ylabel('loss')
plt.xlabel('epochs')
plt.legend()


plt.figure()
plt.plot(results['                  epoch'], results['    metrics/mAP50-95(B)'] * 100)
plt.grid()
plt.title('Validation accuracy vs epochs')
plt.ylabel('accuracy (%)')
plt.xlabel('epochs')

plt.show()