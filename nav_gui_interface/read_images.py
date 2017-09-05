import json
import skimage
from skimage import data, img_as_float
import numpy as np


class ReadGoogleImages():
  def __init__(self, config_file = 'map.json'):
    self.config_file = config_file
    self.images = {}
    self.image = []
    self.jdata = []
  def createImage(self):
    with open(self.config_file) as data_file:    
      data = json.load(data_file)
    self.jdata = data
    num_files = int(data['numfiles'])
    index_mat = np.zeros((data['numtiles_y'], data['numtiles_x']))
    for i in range(num_files):
      image = {}
      print(data[str(i)]['url'])
      image['image'] = img_as_float(skimage.io.imread(data[str(i)]['url'])).copy()
      image['data'] = data[str(i)]
      self.images[i] = image
      index_mat[data[str(i)]['y']][data[str(i)]['x']] = i 
    #concatenate the images in the x axis
    print(index_mat.shape)
    image = {}
    for j in range(data['numtiles_y']):
      image[j] = self.images[index_mat[j][0]]['image']
      for i in range(1, data['numtiles_x']):
        image[j] = np.concatenate((image[j], self.images[index_mat[j][i]]['image']), axis=1)    
        print(index_mat[j][i], j, i)
    #concatenate the images in the y axis
    self.image = image[0]
    for j in range(1, data['numtiles_y']):
      self.image = np.concatenate((self.image, image[j]), axis=0)
    #top = (-data['top'])%256
    #left = (-data['left'])%256
    #self.image = self.image[top: top + height, left:left + width,:]
  def saveImage(self, imageFile='tmp/map.png'):
    skimage.io.imsave(imageFile, self.image)

   
#rimages = ReadGoogleImages()
#rimages.createImage()


#fig = plt.figure("Superpixels")
#ax = fig.add_subplot(1, 1, 1)
#ax.imshow(rimages.image)
#plt.axis("off")
#plt.show()

#with open('map.json') as data_file:    
#      data = json.load(data_file)
