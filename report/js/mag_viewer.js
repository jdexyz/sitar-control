function compareFunction(a,b){
	if(a[2]>b[2]) return 1;
	if(a[2]<b[2]) return -1;
	if(a[1]>b[1]) return 1;
	if(a[1]<b[1]) return -1;
	if(a[0]>b[0]) return 1;
	if(a[0]<b[0]) return -1;
	return 0;
}

$(document).ready(function(){

SCALING = .1;
MAXNORM = 1000;

$(".magneticViewerContainer").each(function(){


var container = $(this);
var title = container.attr("data_title");
var data_url = container.attr("data_url");


	$.ajax({
		url: data_url
	})
	.done(function( csvdata) {

		csvdata.sort(compareFunction);

		var data = [];
		// markers
		var markers = {
		            "marker": {
		                "color": "black", 
		                "size": 1
		            }, 
		            "mode": "markers", 
		            "y": [],
		            "x": [],
		            "z": [], 
		            "type": "scatter3d"
		        };
		for(var i=0; i<csvdata.length; i++){
			if(i%6 ==0){
			    markers.x.push(csvdata[i][0] + SCALING*csvdata[i][3]);
			    markers.y.push(csvdata[i][1] + SCALING*csvdata[i][4]);
			    markers.z.push(csvdata[i][2] + SCALING*csvdata[i][5]);
			}
		}

		data.push(markers);
		console.log("markers ok");

		function heatMapColorforValue(value){
		  var h = (1.0 - value) * 240
		  return "hsl(" + h + ", 100%, 50%)";
		}

		for(var i=0; i<csvdata.length; i++){
			if(i%6 ==0){
		    data.push({
		            "mode": "lines",  
		            "x": [
		                csvdata[i][0],
		                csvdata[i][0] + SCALING*csvdata[i][3]
		            ], 
		            "y": [
		                csvdata[i][1],
		                csvdata[i][1] + SCALING*csvdata[i][4]
		            ],
		            "z": [
		                csvdata[i][2],
		                csvdata[i][2] + SCALING*csvdata[i][5]
		            ], 
		            "line": {
		                "color": heatMapColorforValue(( csvdata[i][3]*csvdata[i][3] +  csvdata[i][4]*csvdata[i][4] +  csvdata[i][5]*csvdata[i][5])/MAXNORM), 
		                "width": 2
		            }, 
		            "type": "scatter3d"
		        });
			}
		}
		console.log("lines ok");

		var figure = {
		    "frames": [], 
		    "layout": {
		        "scene": {
		            "zaxis": {
		                "zerolinecolor": "rgb(255, 255, 255)", 
		                "gridcolor": "rgb(255, 255, 255)", 
		                "showbackground": true, 
		                "backgroundcolor": "rgb(230, 230,230)"
		            }, 
		            "xaxis": {
		                "zerolinecolor": "rgb(255, 255, 255)", 
		                "gridcolor": "rgb(255, 255, 255)", 
		                "showbackground": true, 
		                "backgroundcolor": "rgb(230, 230,230)"
		            }, 
		            "yaxis": {
		                "zerolinecolor": "rgb(255, 255, 255)", 
		                "gridcolor": "rgb(255, 255, 255)", 
		                "showbackground": true, 
		                "backgroundcolor": "rgb(230, 230,230)"
		            }
		        }, 
		        "title": title,
		        "showlegend": false
		    }, 
		    "data": data
		};



		container.click(function(){
			container.addClass("loading");
			setTimeout(function(){
				Plotly.plot(container[0],  {
				    data: figure.data,
				    layout: figure.layout,
				    frames: figure.frames
				    //config: {"mapboxAccessToken": "pk.eyJ1IjoiY2hyaWRkeXAiLCJhIjoiY2lxMnVvdm5iMDA4dnhsbTQ5aHJzcGs0MyJ9.X9o_rzNLNesDxdra4neC_A", "linkText": "Export to plot.ly", "showLink": true}
				});
			},30);
		});
	});

});

});