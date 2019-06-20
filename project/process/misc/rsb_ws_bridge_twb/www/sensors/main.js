 var url = 'ws://' + location.host + '/ws';


var socket_Camera_Infrared = new WebSocket(url);

try {
	socket_Camera_Infrared.onopen = function() {
			var div = document.getElementById("WebsocketCamera");
			div.textContent = "Websocket Verbindung offen";
		}
} catch(exception) {
	alert('<p>Error' + exception);
}

socket_Camera_Infrared.onmessage = function(msg) {  //on receiving a message
	var div = document.getElementById("WebsocketCamera");
	if (msg.data instanceof Blob) { //checking if the received data is a picture
			var canvas = document.getElementById('image');
			var context = canvas.getContext('2d');
			var image = new Image();
			image.onload = function() {
					canvas.width = image.width;
					canvas.height = image.height;
					context.drawImage(image, 0, 0);
			};
			var reader = new FileReader();
			reader.onload = function(e) {
					image.src = e.target.result;
			};
			reader.readAsDataURL(msg.data);
	} 
}

socket_Camera_Infrared.onclose = function() {
		document.getElementById("status").textContent = "Websocket Verbindung geschlossen";
}