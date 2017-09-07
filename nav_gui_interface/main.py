import sys
from PyQt4 import QtCore, QtGui
from PyQt4.Qt import QVariant, QWebSettings, QWebInspector
from PyQt4.QtGui import QImage
from mapUI import Ui_MainWindow
from gmaps import html
from parse_images import ImageInfoParser
import urllib
import json
from read_images import ReadGoogleImages 
from segmentation import Segmentation
from skimage import data, img_as_float
import skimage
import base64
import os.path

global lat, lon
lat = -19.8695362
lon = -43.9645532


class StartQT4(QtGui.QMainWindow):

    def __init__(self, parent=None):
        super(StartQT4, self).__init__(parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        #self.ui.web.page().mainFrame().addToJavaScriptWindowObject('self', self)
        #self.ui.web.setHtml(getHTML(lat, lon))
        self.ui.web.setHtml(html)
        self.ui.web.page().settings().setAttribute(QWebSettings.DeveloperExtrasEnabled, True)
        inspector = QWebInspector()
        inspector.setPage(self.ui.web.page())
        inspector.setVisible(True)


        self.ui.web.page().loadFinished.connect(self.load_finished)


        # here we connect signals with our slots
        self.ctimer = QtCore.QTimer()
        QtCore.QObject.connect(self.ctimer, QtCore.SIGNAL('timeout()'), self.ctimerloop)
        self.ui.okButton.clicked.connect(self.onClick)
        self.imagesParser = ImageInfoParser()
        self.readgoogleimages = ReadGoogleImages()
        self.segmentation = Segmentation()

    
    def onClick(self):
        #Download image informations
        self.saveWebImages()
        #Download and create the image corresponding to the current viewport
        self.readgoogleimages.createImage()

        #image = img_as_float(skimage.io.imread('ex1.jpg'))
        self.segmentation.setImage(self.readgoogleimages.image)
        self.segmentation.segmentImage(self.ui.spinBox.value())
        self.segmentation.buildImageAsBase64()


        #define the parameters for the overlay
        src = '\'' + self.segmentation.base64_segmented_image + '\''

        x_top_left = self.readgoogleimages.jdata['x_top_left'] 
        y_top_left = self.readgoogleimages.jdata['y_top_left'] 

        x_bottom_right = self.readgoogleimages.jdata['x_bottom_right'] 
        y_bottom_right = self.readgoogleimages.jdata['y_bottom_right']
        zoom = self.readgoogleimages.jdata['zoom']


        params = src +','+str(x_top_left) +','+str(y_top_left) +','+ str(x_bottom_right) +','+ str(y_bottom_right) + ','+ str(zoom) 

        #create the map overlay
        self.ui.web.page().mainFrame().evaluateJavaScript('addMapImage('+ params +');')
        self.ui.web.page().mainFrame().evaluateJavaScript("setMap();")

        #define the output json
        self.saveExperimentData()

       

    def saveExperimentData(self, experiment_json = 'experiment.json', map_json = 'map.json'):
        exp_data = {}
        #get obstacles and waypoints from javascrip code
        waypoints = self.ui.web.page().mainFrame().evaluateJavaScript("getWaypoints();").toPyObject()
        obstacles = self.ui.web.page().mainFrame().evaluateJavaScript("getObstacles();").toPyObject()

        exp_data['obstacles'] = obstacles
        exp_data['waypoints'] = waypoints

        #create directory for experiments
        out_dir = 'experiment_config/'
        if not os.path.exists(out_dir):
            os.makedirs(out_dir)

        #save the experiment images to an experiment folder
        self.segmentation.saveAllImages(out_dir)

        exp_data['image_src'] = self.segmentation.rgb_image_file
        exp_data['image_segmented_src'] = self.segmentation.segmented_image_file
        exp_data['segments'] = self.segmentation.segments_file

        #data saved in map file
        data = []
        with open(map_json) as data_file:    
            data = json.load(data_file)
        
        #save other information need for the experiment
        exp_data['height'] = data['height']
        exp_data['width'] = data['width']
        ##position
        exp_data['x_top_left'] = data['x_top_left']
        exp_data['y_top_left'] = data['y_top_left']
        exp_data['zoom'] = data['zoom']

        exp_data['numtiles_x'] = data['numtiles_x']
        exp_data['numtiles_y'] = data['numtiles_y'] 
        #exp_data['image_segmented'] = self.segmentation.segmented_image
        #exp_data['segments'] = self.segmentation.segments
        #exp_data['image'] = self.segmentation.image

        #save image data to json
        with open(out_dir+experiment_json, 'w') as fp:
            json.dump(exp_data, fp)


        #self.ctimer.start(1000)
    def saveWebImages(self, jsonFile = 'map.json', tmpDir = 'tmp/'):

        if not os.path.exists(tmpDir):
            os.makedirs(tmpDir)
        #get the image information from the qtweb
        imageList = self.ui.web.page().mainFrame().evaluateJavaScript('getImages();').toPyObject()
        data = {}
        i = 0
        minx = int(9e16)
        miny = int(9e16)
        maxx = -int(9e16)
        maxy = -int(9e16)
        top = int(9e16)
        left = int(9e16)
        zoom = 19
        #find the image the bounding coordinates of the html images
        for imageinfo in imageList:
            self.imagesParser.feed(str(imageinfo));
            minx = min(minx, self.imagesParser.x)
            miny = min(miny, self.imagesParser.y)
            maxx = max(maxx, self.imagesParser.x)
            maxy = max(maxy, self.imagesParser.y)
            top = min(top, self.imagesParser.top)
            left = min(left, self.imagesParser.left)
            zoom = self.imagesParser.z

        #save the informations in a json
        numtiles_x = 0
        numtiles_y = 0
        for imageinfo in imageList:
            self.imagesParser.feed(str(imageinfo));
            imgpath = tmpDir + str(self.imagesParser.url).split('?')[1] + ".jpg"
            if(not os.path.exists(imgpath)):
                urllib.urlretrieve(str(self.imagesParser.url), imgpath)
            local_data = {}
            url = str(self.imagesParser.url)
            local_data['x'] = self.imagesParser.x - minx
            local_data['y'] = self.imagesParser.y - miny
            local_data['left'] = self.imagesParser.left
            local_data['top'] = self.imagesParser.top
            local_data['url'] = "tmp/"+ str(self.imagesParser.url).split('?')[1] + ".jpg"
            data[i] = local_data
            i = i + 1
            #determine the number of tiles
            numtiles_x = max(numtiles_x, self.imagesParser.x - minx)
            numtiles_y = max(numtiles_y, self.imagesParser.y - miny)

        data['zoom'] = zoom
        data['numfiles'] = i
        data['numtiles_x'] = numtiles_x + 1
        data['numtiles_y'] = numtiles_y + 1
        #
        data['top'] = top
        data['left'] = left
        #
        self.ui.web.page().mainFrame().evaluateJavaScript("setMap();")

        data['height'] = (numtiles_y + 1)*256
        data['width'] = (numtiles_x + 1)*256
        ##position
        data['x_top_left'] = minx
        data['y_top_left'] = miny

        data['x_bottom_right'] = maxx + 1
        data['y_bottom_right'] = maxy + 1

        #save image data to json
        with open(jsonFile, 'w') as fp:
          json.dump(data, fp)


    def load_finished(self):
        print ("Load finished called")
        #print "alert({0});".format("hellow")
        #self.ui.web.page().mainFrame().evaluateJavaScript(("alert('{0}');".format("hellow")))
        #self.ui.web.page().mainFrame().evaluateJavaScript("alert('aaa');")
        #self.ui.web.page().mainFrame().evaluateJavaScript("updateMarkerPos('{0}', '{1}');".format(lat, lon))
     
        pass

    def ctimerloop(self):
        global lat
        lat += 1
        self.ui.web.setHtml(getHTML(lat, lon))
        print ("hi")

if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)
    myapp = StartQT4()
    myapp.show()
    sys.exit(app.exec_())
