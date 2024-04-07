// Get current sensor readings when the page loads  
window.addEventListener('load', getReadings);


const map = new ol.Map({
  layers: [
    new ol.layer.Tile({
      source: new ol.source.TileJSON({
        url: 'https://api.maptiler.com/maps/basic-v2/tiles.json?key=eFukoxoFUFCPGsQBXbyb',
        tileSize: 512,
      })
    })
  ],
  target: 'map',
  view: new ol.View({
    center: ol.proj.fromLonLat([0, 0]),
    zoom: 13
  })
});


const marker = new ol.layer.Vector({
  source: new ol.source.Vector({
    features: [
      new ol.Feature({
        geometry: new ol.geom.Point(
          ol.proj.fromLonLat([0, 0])
        )
      })
    ],
  }),
  style: new ol.style.Style({
    image: new ol.style.Icon({
      src: 'marker-icon.png',
    })
  })
})
map.addLayer(marker)


function changeLatLon(latitude, longitude) {
  // Update the center of the map view
  map.getView().setCenter(ol.proj.fromLonLat([longitude, latitude]));


  // Update the geometry of the marker
  const markerFeature = marker.getSource().getFeatures()[0];
  const markerGeometry = markerFeature.getGeometry();
  markerGeometry.setCoordinates(ol.proj.fromLonLat([longitude, latitude]));
}


// Create Temperature Gauge
var gaugeBPM = new LinearGauge({
  renderTo: 'gauge-heartrate',
  width: 120,
  height: 400,
  units: "BPM",
  minValue: 0,
  maxValue: 220,
  majorTicks: [
      "0",
      "20",
      "40",
      "60",
      "80",
      "100",
      "120",
      "140",
      "160",
      "180",
      "200",
      "220"
  ],
  minorTicks: 2,
  strokeTicks: true,
  highlights: [
      {
          "from": 120,
          "to": 220,
          "color": "rgba(200, 50, 50, .75)"
      }
  ],
  colorPlate: "#fff",
  borderShadowWidth: 0,
  borders: false,
  needleType: "arrow",
  needleWidth: 2,
  animationDuration: 1500,
  animationRule: "linear",
  tickSide: "left",
  numberSide: "left",
  needleSide: "left",
  barStrokeWidth: 7,
  barBeginCircle: false,
  value: 75
}).draw();
 
// Create Humidity Gauge
var gaugeSpO2 = new RadialGauge({
  renderTo: 'gauge-SpO2',
  width: 300,
  height: 300,
  units: "SpO2%",
  minValue: 0,
  maxValue: 100,
  majorTicks: [
      "0",
      "10",
      "20",
      "30",
      "40",
      "50",
      "60",
      "70",
      "80",
      "90",
      "100"
  ],
  minorTicks: 5,
  strokeTicks: true,
  highlights: [
      {
          "from": 0,
          "to": 90,
          "color": "rgba(200, 50, 50, .75)"
      },
      {
          "from": 90,
          "to": 100,
          "color": "#3a3"
      }
  ],
  colorPlate: "#fff",
  borderShadowWidth: 0,
  borders: false,
  needleType: "arrow",
  needleWidth: 2,
  needleCircleSize: 7,
  needleCircleOuter: true,
  needleCircleInner: false,
  animationDuration: 1500,
  animationRule: "linear"
}).draw();


// Function to get current readings on the webpage when it loads for the first time
function getReadings(){
  var xhr = new XMLHttpRequest();
  xhr.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      var myObj = JSON.parse(this.responseText);
      console.log(myObj);
      var heartrate = myObj.heartrate;
      var SpO2 = myObj.SpO2;
      var steps = myObj.steps;
      var latitude = myObj.latitude;
      var longitude = myObj.longitude;
      gaugeBPM.value = heartrate;
      gaugeHum.value = SpO2;
      document.getElementById("steps").innerHTML = steps;
      changeLatLon(latitude, longitude);
    }
  };
  xhr.open("GET", "/readings", true);
  xhr.send();
}


if (!!window.EventSource) {
  var source = new EventSource('/events');
 
  source.addEventListener('open', function(e) {
    console.log("Events Connected");
  }, false);


  source.addEventListener('error', function(e) {
    if (e.target.readyState != EventSource.OPEN) {
      console.log("Events Disconnected");
    }
  }, false);
 
  source.addEventListener('message', function(e) {
    console.log("message", e.data);
  }, false);
 
  source.addEventListener('new_readings', function(e) {
    console.log("new_readings", e.data);
    var myObj = JSON.parse(e.data);
    console.log(myObj);
    gaugeBPM.value = myObj.heartrate;
    gaugeSpO2.value = myObj.SpO2;
    var latitude = myObj.latitude;
    var longitude = myObj.longitude;
    document.getElementById("steps").innerHTML = myObj.steps;
    changeLatLon(latitude, longitude);
  }, false);
}