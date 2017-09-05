#!/usr/bin/python

html = """
<!DOCTYPE html>
    <html>
      <head>
        <style>
          html, body, #map-canvas {
            height: 100%;
            margin: 0px;
            padding: 0px
          }
        </style>
        <script src="https://maps.googleapis.com/maps/api/js?v=3.exp&sensor=false&libraries=drawing"></script>
        <script src="http://google-maps-utility-library-v3.googlecode.com/svn/trunk/markerwithlabel/src/markerwithlabel.js"></script>
        <script language="JavaScript">
          var polylines = [];
          var waypoint = [];
          var map = 'undefined';
          var overlay;
          USGSOverlay.prototype = new google.maps.OverlayView();
          var imagedata = ''

            /* Initialize function */
          /**************************************************************************/
          function initialize() {
            var mapOptions = {
              center: new google.maps.LatLng(-19.8695912, -43.9583309),
              //center: {lat: 62.323907, lng: -150.109291},
              zoom: 19,
              tilt: 0,
              heading: 180,
              mapTypeId: google.maps.MapTypeId.SATELLITE
            };
            map = new google.maps.Map(document.getElementById('map-canvas'), mapOptions);
            drawingManager.setMap(map);

            //google.maps.event.addListener(drawingManager, 'polylinecomplete', function(polyline) {
            //  polylines.push(polyline);
            //});
            google.maps.event.addListener(drawingManager, 'polygoncomplete', function(polyline) {
              polylines.push(polyline);
            });
            


            var bounds = new google.maps.LatLngBounds(
              new google.maps.LatLng(62.281819, -150.287132),
              new google.maps.LatLng(62.400471, -150.005608));

            overlay = new USGSOverlay(map);
          }
          /**************************************************************************/



          /* Waypoints functions */
          /**************************************************************************/
          function pixelToLatlng(xcoor, ycoor) {
            var ne = map.getBounds().getNorthEast();
            var sw = map.getBounds().getSouthWest();
            var projection = map.getProjection();
            var topRight = projection.fromLatLngToPoint(ne);
            var bottomLeft = projection.fromLatLngToPoint(sw);
            var scale = 1 << map.getZoom();
            var newLatlng = projection.fromPointToLatLng(new google.maps.Point(xcoor / scale + bottomLeft.x, ycoor / scale + topRight.y));

            //marker.setPosition(newLatlng);
            return newLatlng;
          };

          function initWaypointMarker(size){
            var i;
            for (i = 0; i < size; i++) {
              var i_str = (i+1).toString();
              waypoint.push(new MarkerWithLabel({
                position: new google.maps.LatLng(-19.8695912, -43.9583309),
                title: i_str,
                labelContent: i_str,
                labelInBackground: false,
                map: map
              }));
            }
          }

          function updateMarkerWaypoint(px, py){
            var i;
            if (waypoint.empty) return;
            for (i = 0; i < waypoint.length; i++) {
              newPos = pixelToLatlng(px[i], py[i])
              waypoint[i].setPosition(newPos);
            }
          }

          function getWaypointPloted(){
            if (waypoint.empty) return;
            var wpLatLng = [];
            for (var i = 0; i < waypoint.length; i++){
              wpLatLng.push([waypoint[i].getPosition().lat(), waypoint[i].getPosition().lng()] );
            }
            return wpLatLng;
          }
          
          function getImages(){
            //map.setCenter(marker.getPosition());
            var imagesList = [];
            divs = map.getDiv().getElementsByTagName('div');
            for(var i = 0; i < divs.length; i++){
              if(divs[i].firstChild != null){
                if(divs[i].firstChild.localName == 'img'){
                  if(divs[i].outerHTML.indexOf("khms") !== -1){
                    imagesList.push(divs[i].outerHTML);
                  }
                }
              }
            }

            return imagesList;
          }

          var drawingManager = new google.maps.drawing.DrawingManager({
             drawingMode: google.maps.drawing.OverlayType.MARKER,
             drawingControl: true,
             drawingControlOptions: {
                position: google.maps.ControlPosition.TOP_CENTER,
                drawingModes: ['marker', 'circle', 'polygon', 'polyline', 'rectangle']
              },
             markerOptions: {icon: 'https://developers.google.com/maps/documentation/javascript/examples/full/images/beachflag.png'},
             circleOptions: {
             fillColor: '#ffff00',
             fillOpacity: 1,
             strokeWeight: 5,
             clickable: true,
             draggable: true,
             editable: true,
             zIndex: 1
            },

            polylineOptions: {
             fillColor: '#ffff00',
             fillOpacity: 1,
             strokeWeight: 5,
             clickable: true,
             draggable: true,
             editable: true,
             zIndex: 1
            }
          });
        
          



          /*********************************OVERLAY*****************************************/

         
          /** @constructor */
          function USGSOverlay(map) {

            // Initialize all properties.
            this.map_ = map;

            // Define a property to hold the image's div. We'll
            // actually create this div upon receipt of the onAdd()
            // method so we'll leave it null for now.
            this.div_ = null;
            
          }
          function addMapImage(base64image, x_top_left, y_top_left, x_bottom_right, y_bottom_right, zoom){

            projection = map.getProjection();

            //convert the tile coordinates to a normalized 0-256
            x_tl = (x_top_left/(Math.pow(2, zoom)))*256
            y_tl = (y_top_left/(Math.pow(2, zoom)))*256

            x_br = (x_bottom_right/(Math.pow(2, zoom)))*256
            y_br = (y_bottom_right/(Math.pow(2, zoom)))*256
            
            //get the normalized tile bounds and convert to lng and lat
            top_left = projection.fromPointToLatLng(new google.maps.Point(x_tl, y_br));
            bottom_right = projection.fromPointToLatLng(new google.maps.Point(x_br, y_tl));
            overlay.bounds_ = new google.maps.LatLngBounds(top_left, bottom_right);
            
            //imagedata
            imagedata = "data:image/jpeg;base64," + base64image;
          }

          function setMap(){
            overlay.setMap(map);
          }

          /**
           * onAdd is called when the map's panes are ready and the overlay has been
           * added to the map.
           */
          USGSOverlay.prototype.onAdd = function() {

            var div = document.createElement('div');
            div.style.borderStyle = 'none';
            div.style.borderWidth = '0px';
            div.style.position = 'absolute';

            // Create the img element and attach it to the div.
            var img = document.createElement('img');
            img.src = imagedata;
            img.style.width = '100%';
            img.style.height = '100%';
            img.style.position = 'absolute';
            div.appendChild(img);

            this.div_ = div;

            // Add the element to the "overlayLayer" pane.
            var panes = this.getPanes();
            panes.overlayLayer.appendChild(div);
          };

          USGSOverlay.prototype.draw = function() {
            
            // We use the south-west and north-east
            // coordinates of the overlay to peg it to the correct position and size.
            // To do this, we need to retrieve the projection from the overlay.
            var overlayProjection = this.getProjection();

            // Retrieve the south-west and north-east coordinates of this overlay
            // in LatLngs and convert them to pixel coordinates.
            // We'll use these coordinates to resize the div.
            var sw = overlayProjection.fromLatLngToDivPixel(this.bounds_.getSouthWest());
            var ne = overlayProjection.fromLatLngToDivPixel(this.bounds_.getNorthEast());

            // Resize the image's div to fit the indicated dimensions.
            var div = this.div_;
            div.style.left = sw.x + 'px';
            div.style.top = ne.y + 'px';
            div.style.width = (ne.x - sw.x) + 'px';
            div.style.height = (sw.y - ne.y) + 'px';
            div.style.opacity = 0.8;
            
          };

          // The onRemove() method will be called automatically from the API if
          // we ever set the overlay's map property to 'null'.
          USGSOverlay.prototype.onRemove = function() {
            this.div_.parentNode.removeChild(this.div_);
            this.div_ = null;
          };


          google.maps.event.addDomListener(window, 'load', initialize);
        </script>
      </head>

      <body>
        <div id="map-canvas"></div>
      </body>
    </html>
"""
