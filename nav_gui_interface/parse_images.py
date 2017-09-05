from HTMLParser import HTMLParser
import re

# create a subclass and override the handler methods
class ImageInfoParser(HTMLParser):
  def __init__(self):
    HTMLParser.__init__(self)
    self.left = 0
    self.top = 0
    self.x = 0
    self.y = 0
    self.z = 0
    self.url = ''
  def handle_starttag(self, tag, attrs):
    for attr in attrs:
      if(tag == 'div' and attr[0] == 'style'):
         atr_string = attr[1].split(';')
         self.left = int(re.findall("[-+]?[.]?[\d]+(?:,\d\d\d)*[\.]?\d*(?:[eE][-+]?\d+)?", atr_string[1])[0])
         self.top = int(re.findall("[-+]?[.]?[\d]+(?:,\d\d\d)*[\.]?\d*(?:[eE][-+]?\d+)?", atr_string[2])[0])
          

      if(tag == 'img' and attr[0] == 'src'):
         self.url = attr[1]
         self.x = int(self.url.split("x=")[1].split("&")[0])
         self.y = int(self.url.split("y=")[1].split("&")[0])
         self.z = int(self.url.split("z=")[1].split(".")[0])



#parser = MyHTMLParser()
#parser.feed('<div style="position: absolute; left: 125px; top: -69px; -webkit-transition: opacity 200ms ease-out;"><img style="width: 256px; height: 256px; -webkit-user-select: none; border: 0px; padding: 0px; margin: 0px; max-width: none;" src="https://khms0.googleapis.com/kh?v=733&amp;hl=en-US&amp;x=198124&amp;y=291678&amp;z=19" draggable="false" alt=""/></div>')
