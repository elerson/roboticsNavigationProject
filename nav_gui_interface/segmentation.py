import numpy as np
import matplotlib.pyplot as plt
import skimage
from skimage import data, img_as_float
from skimage.segmentation import felzenszwalb, slic, quickshift, watershed, chan_vese
from skimage.segmentation import mark_boundaries
import cv2
import base64


class Segmentation:
  def __init__(self, tmpdir = 'tmp/'):
    self.image = []
    self.segmented_image = []
    self.segments = []
    self.colored_segments = []
    self.segments_colors = []
    self.generateColors__()
    self.base64_image = ''
    self.base64_segmented_image = ''
    self.tmpdir = tmpdir
  def segmentImage(self, level = 10000):
    self.segments = felzenszwalb(self.image, level)
    self.segmented_image = mark_boundaries(img_as_float(self.image), self.segments)
    self.getColoredSegments__()
  def setImage(self, image):
    self.image = image
  def generateColors__(self, n_colors = 3000):
    segments_nmb = n_colors
    # generate (r,g,b) color values
    for ind in range(segments_nmb):
      r = np.random.randint(255)
      g = np.random.randint(255)
      b = np.random.randint(255)
      self.segments_colors.append((r,g,b))
  def getColoredSegments__(self):
    self.colored_segments = np.zeros([self.segments.shape[0],self.segments.shape[1],3], dtype=np.uint8)
    # segment labels which need to be displayed
    labels = set(np.unique(self.segments)).difference((0,))
    for lb in labels:
      segment_mask = (self.segments == lb)
      self.colored_segments[:,:,0][ segment_mask.nonzero() ] = self.segments_colors[int(lb)][0]
      self.colored_segments[:,:,1][ segment_mask.nonzero() ] = self.segments_colors[int(lb)][1]
      self.colored_segments[:,:,2][ segment_mask.nonzero() ] = self.segments_colors[int(lb)][2]
  def buildImageAsBase64(self):
    skimage.io.imsave(self.tmpdir+'rgbimage.jpg', self.image)
    skimage.io.imsave(self.tmpdir+'segmentedimage.jpg', self.colored_segments)

    with open(self.tmpdir+'rgbimage.jpg', "rb") as imageFile:
      self.base64_image = base64.b64encode(imageFile.read())

    with open(self.tmpdir+'segmentedimage.jpg', "rb") as imageFile:
      self.base64_segmented_image = base64.b64encode(imageFile.read())


#segmentation = Segmentation()
#segmentation.setImage(image)
#segmentation.segmentImage()
#segmentation.buildImageAsBase64()

#viewer = ImageViewer(segmentation.colored_segments)
#viewer.show()




#image = img_as_float(skimage.io.imread('ex1.jpg'))
# Feel free to play around with the parameters to see how they impact the result
#cv = chan_vese(image, mu=0.25, lambda1=1, lambda2=1, tol=1e-3, max_iter=200,
#               dt=0.5, init_level_set="checkerboard", extended_output=True)
#segments = felzenszwalb(image, 100)
#segments = quickshift(image, 1.0, kernel_size=5, max_dist=100)#skimage.segmentation.felzenszwalb(image)

# show the output of SLIC
#fig = plt.figure("Superpixels")
#ax = fig.add_subplot(1, 1, 1)
#ax.imshow(mark_boundaries(img_as_float(image), segments))
#plt.axis("off")
#plt.show()


#fig = plt.figure("Superpixels")
#ax = fig.add_subplot(1, 1, 1)
#ax.imshow(i3)
#plt.axis("off")
#plt.show()


#fig, axes = plt.subplots(2, figsize=(8, 8))
#ax = axes.flatten()

#ax[0].imshow(image)
#ax[0].set_axis_off()
#ax[0].set_title("Original Image", fontsize=12)

#ax[1].imshow(cv[0])
#ax[1].set_axis_off()
#title = "Chan-Vese segmentation - {} iterations".format(len(cv[2]))
#ax[1].set_title(title, fontsize=12)

#ax[2].imshow(cv[1], cmap="gray")
#ax[2].set_axis_off()
#ax[2].set_title("Final Level Set", fontsize=12)

#ax[3].plot(cv[2])
#ax[3].set_title("Evolution of energy over iterations", fontsize=12)

#fig.tight_layout()
#plt.show()
