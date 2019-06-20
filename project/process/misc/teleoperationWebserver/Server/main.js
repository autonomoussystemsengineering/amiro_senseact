/*
 * Author: Markus Gaffke
 * File: main.js
 * Purpose: connecting to Server and running the Webside for bouth WS
 * Last Changed: 05.07.2014
 *
 */


//====================================================Funktions & Variables for all Infrared Canvases=============================
var colorcode ="00ff00"; 				//setting the start colorcode for infaredcones to green

var countercam=0, counterinf=0;
//================================================================GETING URL==================================================
function get_appropriate_ws_url()
{
	var pcol;
	var u = document.URL;

	/*
	 * We open the websocket encrypted if this page came on an
	 * https:// url itself, otherwise unencrypted
	 */

	if (u.substring(0, 5) == "https") {
		pcol = "wss://";
		u = u.substr(8);
	} else {
		pcol = "ws://";
		if (u.substring(0, 4) == "http")
			u = u.substr(7);
	}

	u = u.split('/');

	/* + "/xxx" bit is for IE10 workaround */

	return pcol + u[0] + "/xxx";
};

var url = "ws://" + document.location.host;

//=======================================================================CAMERA=============================================================
var socket_camera;
socket_camera = new WebSocket(url, "camera");
var speedbar_old;
var speed_cam_send;
var speedb = document.getElementById("speedbar");
var robotIDcheck =  document.getElementById("Robot_ID");
 var canvas = document.getElementById('image');
try {
         socket_camera.onopen = function() {
            var div = document.getElementById("WebsocketCamera");
            div.textContent = " websocket connection opened ";
	    

         }

        socket_camera.onmessage = function(msg) { //on receiving a message
		if(in_control && !controlpannel_free){
			showControlpannel();
		controlpannel_free=true;
		}
		//if in steering control, display more Information
		if(in_control){	document.getElementById("Speed_Status").textContent = "Geschwindigkeit: " + speedb.value + "%";
			document.getElementById("speeddiv").textContent = "Geschwindigkeit: "+ speedb.value + "%";
			document.getElementById("img_NO_CONTROL").style.zIndex='-10';			
			}
			if(speedb.value != speedbar_old && in_control){
			socket_camera.send(speedb.value);
			speedbar_old = speedb.value;			
			}
			if(speed_cam_send && speedIMGX >= 0 && in_control){
			socket_camera.send(200+speedIMGX);
			socket_camera.send(500+speedIMGY);				
			stop_signal_send = false;
			}
            var div = document.getElementById("WebsocketCamera");
            if (msg.data instanceof Blob) { //checking if the received data is a picture
		//countercam++;
		//document.getElementById("Mouse_Position").textContent = "Counter Cam / Inf: " + countercam + " / " + counterinf;
               
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
                div.textContent = "Kamera Websocket verbunden";
            } else {
		div.textContent = "Camera Websocket Wrong message received";
	    }
        };
        
        socket_camera.onclose = function() {
            document.getElementById("WebsocketCamera").textContent = "Kamera Websocket-Verbindung geschlossen";
	    
        };
} catch(exception) {
	alert('<p>ErrorHere' + exception);
}



function showControlpannel(){
		document.getElementById("imgnormal").style.visibility ="visible";
		document.getElementById("img_NO_CONTROL").style.visibility ="hidden";
		document.getElementById("img_OL").style.visibility ="visible";
		document.getElementById("img_O").style.visibility ="visible";
		document.getElementById("img_OR").style.visibility ="visible";
		document.getElementById("img_MR").style.visibility ="visible";
		document.getElementById("img_ML").style.visibility ="visible";
		document.getElementById("img_UR").style.visibility ="visible";
		document.getElementById("img_U").style.visibility ="visible";
		document.getElementById("img_UL").style.visibility ="visible";
};
//==================================================================Mouscontrol=============================================================

      function getMousePos(canvas_control, evt) 
{
	var rect = canvas_control.getBoundingClientRect();
	return {x: evt.clientX - rect.left , y: evt.clientY - rect.top }

};



 var canvas_img_control = document.getElementById('image');
 var mousePos, mousePosDownX, mousePosDownY, speedIMGX, speedIMGY;
 var mouseSteeringOn, stop_signal_send;
 var imgData = document.getElementById("imgnormal");


function setControlImgBack(){
	document.getElementById("img_OL").style.zIndex='-1';
	document.getElementById("img_O").style.zIndex='-2';
	document.getElementById("img_OR").style.zIndex='-3';
	document.getElementById("img_ML").style.zIndex='-4';
	document.getElementById("img_MR").style.zIndex='-5';
	document.getElementById("img_UL").style.zIndex='-6';
	document.getElementById("img_U").style.zIndex='-7';
	document.getElementById("img_UR").style.zIndex='-8';
	if(in_control){socket_camera.send(1099);} // send stop Signal
};



function getSpeed(speedterm){
	var rangeLimit = 200;	
	if(speedterm>rangeLimit){speedterm=rangeLimit}
	if(-speedterm>rangeLimit){speedterm=-rangeLimit}
	var actualSpeed=(speedterm/rangeLimit)*100;
return parseInt(actualSpeed);
};


//=========================================================LIGHT BUTTONS==================================================================

window.onload = function() {
   	document.getElementById("whiteLight").onclick = function() {
		if(in_control){	
			if(whiteLight==0){	
                        allLightsOut()
                        document.getElementById("whiteLight").value="Weißes Licht an";
                        getLightMessage();
                        whiteLight=1;
                        socket_camera.send(1105);
                        socket_camera.send(1101);
                        //document.getElementById("bebot_div").style.visibility = "visible";
                        //bebot = true;
                        //document.getElementById("amiro_div").style.visibility = "hidden";
                        }else{
                        allLightsOut()
                        getLightMessage();
                        //document.getElementById("bebot_div").style.visibility = "hidden";
                        //bebot = false;
                        //document.getElementById("amiro_div").style.visibility = "visible";
                        whiteLight=0;
                        socket_camera.send(1105);
			}
		}
	};	


   	document.getElementById("redLight").onclick = function() {
		if(in_control){
                        if(redLight==0){
                        allLightsOut()
                        document.getElementById("redLight").value="Rotes Licht an";
                        getLightMessage();
                        redLight=1;
                        socket_camera.send(1105);
                        socket_camera.send(1102);
                        }else{
                        allLightsOut()
                        getLightMessage();
                        redLight=0;
                        socket_camera.send(1105);
			}
		}
	};	


   	document.getElementById("greenLight").onclick = function() {
		if(in_control){
                        if(greenLight==0){
                        allLightsOut()
                        document.getElementById("greenLight").value="Gruenes Licht an";
                        getLightMessage();
                        greenLight=1;
                        socket_camera.send(1105);
                        socket_camera.send(1103);
                        }else{
                        allLightsOut()
                        getLightMessage();
                        greenLight=0;
                        socket_camera.send(1105);
			}
		}
	};	


   	document.getElementById("blueLight").onclick = function() {
		if(in_control){		
                        if(blueLight==0){
			allLightsOut()
                        document.getElementById("blueLight").value="Blaues Licht an";
                        getLightMessage();
                        blueLight=1;
                        socket_camera.send(1105);
                        socket_camera.send(1104);
                        }else{
                        allLightsOut()
                        getLightMessage();
                        blueLight=0;
                        socket_camera.send(1105);
			}
		}
	};

	document.getElementById("leave_get_Control").onclick = function() {
		// if true, give up control over robot		
		if(in_control && total_Number_of_Clients !== 1){
			in_control = false;
			User_Number = User_Count +1;
			document.getElementById("leave_get_Control").value="In Warteschlange";
			document.getElementById("User_Number").textContent = "Deine Usernummer: " + User_Number;
			document.getElementById("img_NO_CONTROL").style.visibility ="visible";
			document.getElementById("img_NO_CONTROL").style.zIndex ="10";
			document.getElementById("imgnormal").style.visibility='hidden';		
			document.getElementById("img_OL").style.visibility='hidden';
			document.getElementById("img_O").style.visibility='hidden';
			document.getElementById("img_OR").style.visibility='hidden';
			document.getElementById("img_ML").style.visibility='hidden';
			document.getElementById("img_MR").style.visibility='hidden';
			document.getElementById("img_UL").style.visibility='hidden';
			document.getElementById("img_U").style.visibility='hidden';
			document.getElementById("img_UR").style.visibility='hidden';
			socket_camera.send(1108);
		} 
		
	};

	document.getElementById("help").onclick = function() {
		document.getElementById("help_Example").style.visibility='visible';
		document.getElementById("help_div").style.visibility='visible';
		document.getElementById("img_Help").style.visibility='visible';
	};
	

	
};
	

function allLightsOut(){
        document.getElementById("whiteLight").value="Weißes Licht aus";
        document.getElementById("redLight").value="Rotes Licht aus";
        document.getElementById("greenLight").value="Gruenes Licht aus";
        document.getElementById("blueLight").value="Blaues Licht aus";

};


function getLightMessage(){
	var lightMessage;
	var white=" an /", red=" an /", green=" an /",blue=" an";	
	if(whiteLight == 0){white="aus /";}
	if(redLight == 0){red="aus /";}
	if(greenLight == 0){green="aus /";}
	if(blueLight == 0){blue="aus";}
	lightMessage="Licht: Weiß: " + white + " Rot: " + red + " Gruen: " + green + " Blau: " + blue;
		
	document.getElementById("Light_Status").textContent =lightMessage;
};





//==============================================================INFRARED=============================================================

var socket_infrared;
	var num_counter =0;
	if (typeof MozWebSocket != "undefined") {
		socket_infrared = new MozWebSocket(get_appropriate_ws_url(),
				   "infrared");
	} else {
		socket_infrared = new WebSocket(get_appropriate_ws_url(),
				   "infrared");
	}


	try {
		socket_infrared.onopen = function() {
		document.getElementById("WebsocketInfrared").textContent = "Infrarot Websocket verbunden";
		document.getElementById("Light_Status").textContent ="Licht: aus";
			
		}; 

		socket_infrared.onmessage =function got_packet(info) {	
				
			var infraredData = info.data.split(":"); // getting the InfraredSensor Information out of the received data
			var whoIsInControlAndLight = info.data.split(";"); // getting the User and Light Information out of the received
			document.getElementById("User_Robot_Control").textContent = "Nutzer in Kontrolle: " 
				+ whoIsInControlAndLight[1];
			document.getElementById("User_Headcount").textContent = " Anzahl Nutzer:" +whoIsInControlAndLight[3];
			User_Count = parseInt(whoIsInControlAndLight[2]);
			total_Number_of_Clients = parseInt(whoIsInControlAndLight[3]);
			if(isChecked == false){
				//receiving User number
				User_Number = parseInt(whoIsInControlAndLight[2]);
				document.getElementById("User_Number").textContent = "Deine Usernummer: " + User_Number;
				isChecked = true;
				if(parseInt(infraredData[0]) ==1 ){ //checking which robot the system is running on
				document.getElementById("Robot_ID").textContent = "Roboter: Bebot";
				document.getElementById("bebot_div").style.visibility = "visible";
				document.getElementById("amiro_div").style.visibility = "hidden";
				bebot = true;
				} else {
				document.getElementById("Robot_ID").textContent = "Roboter: Amiro";
				document.getElementById("amiro_div").style.visibility = "visible";
				document.getElementById("bebot_div").style.visibility = "hidden";
				}
			}
			socket_infrared.send(User_Number);
		
			if(User_Number == whoIsInControlAndLight[1]){ //checking if User is in control
			document.getElementById("leave_get_Control").value="Kontrolle abgeben";
			in_control = true;
			showControlpannel();
			document.getElementById("Steering_Status").textContent = "Du darfst steuern!";			
			}else{
			document.getElementById("Steering_Status").textContent = "Der Nutzer mit der Nummer "+ whoIsInControlAndLight[1] 
			+ " darf steuern!";
			}
//=============================================Updating Light Information======================================================
			whiteLight=parseInt(whoIsInControlAndLight[4]);
			redLight=parseInt(whoIsInControlAndLight[5]);
			greenLight=parseInt(whoIsInControlAndLight[6]);
			blueLight=parseInt(whoIsInControlAndLight[7]);
			getLightMessage();

//=============================================Infrared color update for Bebot======================================================
		if(bebot) {						
			updateBeBot(infraredData);
			}
//=============================================Infrared color update for Amiro==========================================================
		if(!bebot){						
			updateAmiro(infraredData);
			}
            };
		 

		socket_infrared.onclose = function(){
			document.getElementById("WebsocketInfrared").textContent = "Infrarot Websocket Verbindung geschlosssen";
		};
	

	} catch(exception) {
		alert('<p>Error' + exception);  
	}

/*
*Converting InfraredData into a colorcode and returning this colorcode 
*
*/	
function getCode(sensorInt){
			var color_code = getGreenToRed(parseInt(sensorInt));
	   		var code_split = color_code.split(",");		 	
	   		return  RGBtoHex(code_split[1], code_split[2], code_split[3]);  	       
	};


//http://stackoverflow.com/questions/7128675/from-green-to-red-color-depend-on-percentage
function getGreenToRed(percent){
           var g = percent<50 ? 255 : Math.floor(255-(percent*2-100)*255/100);
           var r = percent>50 ? 255 : Math.floor((percent*2)*255/100);
	return 'rgb(,'+r+','+g+',0)';  	
        };


//http://www.linuxtopia.org/online_books/javascript_guides/javascript_faq/rgbtohex.htm
function RGBtoHex(R,G,B) {return toHex(R)+toHex(G)+toHex(B)}
function toHex(N) {
 if (N==null) return "00";
 N=parseInt(N); if (N==0 || isNaN(N)) return "00";
 N=Math.max(0,N); N=Math.min(N,255); N=Math.round(N);
 return "0123456789ABCDEF".charAt((N-N%16)/16)
      + "0123456789ABCDEF".charAt(N%16);
};



