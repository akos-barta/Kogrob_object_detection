from gluoncv import model_zoo, data, utils
from matplotlib import pyplot as plt
from PIL import Image
import mxnet as mx
import numpy as np
import time
import cv2

net = model_zoo.get_model('ssd_512_resnet50_v1_voc', pretrained=True)

start_time = time.time()

image = cv2.imread('street_small.jpg')

image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
mx_im = mx.nd.array(image_rgb)
#image=mx.image.imread('street_small.jpg')

x, img = data.transforms.presets.ssd.transform_test(mx_im,short=512)

class_IDs, scores, bounding_boxes = net(x)

ax = utils.viz.plot_bbox(img, bounding_boxes[0], scores[0],
                         class_IDs[0], class_names=net.classes)
ax.axis('off')
plt.tight_layout()

fig = ax.get_figure()
fig.canvas.draw()
img_data = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
img_data = img_data.reshape(fig.canvas.get_width_height()[::-1] + (3,))
pil_img = Image.fromarray(img_data)

output_file = 'output.jpg'
pil_img.save(output_file, 'JPEG')

# Stop the timer
end_time = time.time()

# Calculate the elapsed time
elapsed_time = end_time - start_time

# Print the elapsed time
print(f"Elapsed time: {elapsed_time} seconds")


