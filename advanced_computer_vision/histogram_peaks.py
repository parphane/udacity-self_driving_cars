import numpy as np
import matplotlib.image as mpimg
import matplotlib.pyplot as plt

# Load our image
# `mpimg.imread` will load .jpg as 0-255, so normalize back to 0-1
img = mpimg.imread('warped_example.jpg') / 255


def hist(img):
    # TO-DO: Grab only the bottom half of the image
    # Lane lines are likely to be mostly vertical nearest to the car
    bottom_half = img[img.shape[0] // 2:, :]

    # TO-DO: Sum across image pixels vertically - make sure to set `axis`
    # i.e. the highest areas of vertical lines should be larger values
    histogram = np.sum(bottom_half, axis=0)

    return histogram


# Create histogram of image binary activations
histogram = hist(img)

# Visualize the resulting histogram
f, (ax1, ax2) = plt.subplots(1, 2, figsize=(24, 9))

ax1.imshow(img)
ax1.set_title('Original Image', fontsize=50)
ax2.plot(histogram)
ax2.set_title('Histogram Image', fontsize=50)
plt.show()
