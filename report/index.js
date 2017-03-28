var express = require('express');
var fs = require('fs');
var app = express();
var path = require('path');


app.set('port', (process.env.PORT || 80));

app.use('/media', express.static(__dirname + '/media'));
app.use('/css', express.static(__dirname + '/css'));
app.use('/js', express.static(__dirname + '/js'));
app.use('/img', express.static(__dirname + '/img'));
app.use('/json', express.static(__dirname + '/json'));
app.use('/public', express.static(__dirname + '/public'));

var footer;
fs.readFile("./footer.html", function(err, data){
    if (err) return console.error(err);
    footer = data;
});
var header;
fs.readFile("./header.html", function(err, data){
    if (err) return console.error(err);
    header = data;
});


var routes = {
	"/" : "intro",
	"/about" : "about",
	"/choosing_sensors" : "choosing_sensors",
	"/control" : "control",
	"/dynamics" : "dynamics",
	"/conclusion" : "conclusion",
	"/general_actuators" : "general_actuators",
	"/intro" : "intro",
	"/landscape" : "landscape",
	"/linearising" : "linearising",
	"/magnetic_scanner" : "magnetic_scanner",
	"/magnetic_scanner_design" : "magnetic_scanner_design",
	"/magnetic_scanner_results" : "magnetic_scanner_results",
	"/making_sitemp" : "making_sitemp",
	"/state_sensor_algorithm" : "state_sensor_algorithm",
	"/comparing" : "comparing",
	"/website" : "website",
	"/summary_design" : "summary_design",
	"/general" : "general",
};


for (var key in routes){
	(function(file, key) {
		var content = fs.readFileSync('./html/' + file + '.html'); // Volontairly blocking, executed once at start up. And yes, it is dirty.
		app.get(key, function(request, response) {
			response.set('Content-Type', 'text/html');
			response.send(header + content + footer);
		});
    })(routes[key], key);
}



app.listen(app.get('port'), function() {
  console.log('Node app is running on port', app.get('port'));
});
