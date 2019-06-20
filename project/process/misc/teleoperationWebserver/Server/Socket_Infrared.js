 
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
		document.getElementById("WebsocketInfrared").textContent = "Infrarot Websocket: CONNECTED";
		document.getElementById("Light_Status").textContent ="Licht: aus";
			
		} 

		socket_infrared.onmessage =function got_packet(info) {	
				
			var infraredData = info.data.split(":");
			var whoIsInControlAndLight = info.data.split(";");
			document.getElementById("User_Robot_Control").textContent = "Client in Control: " 
				+ whoIsInControlAndLight[1];
			document.getElementById("User_Headcount").textContent = " Anzahl Clients:" +whoIsInControlAndLight[3];
			User_Count = parseInt(whoIsInControlAndLight[2]);
			total_Number_of_Clients = parseInt(whoIsInControlAndLight[3]);
			if(isChecked == false){
				isChecked = true;
				if(parseInt(infraredData[0]) ==1 ){
				document.getElementById("Robot_ID").textContent = "Roboter: Bebot";
				document.getElementById("bebot_div").style.visibility = "visible";
				bebot = true;
				document.getElementById("amiro_div").style.visibility = "hidden";
				User_Number = parseInt(whoIsInControlAndLight[2]);
				document.getElementById("User_Number").textContent = "Deine Usernummer: " + User_Number;
				} else {
				document.getElementById("Robot_ID").textContent = "Roboter: Amiro";
				document.getElementById("amiro_div").style.visibility = "visible";
				document.getElementById("bebot_div").style.visibility = "hidden";
				}
			}
			socket_infrared.send(User_Number);
		
			if(User_Number == whoIsInControlAndLight[1]){
			document.getElementById("leave_get_Control").value="Kontrolle abgeben";
			in_control = true;
			showControlpannel();
			document.getElementById("Steering_Status").textContent = "Du darfst steuern!";			
			}else{
			document.getElementById("Steering_Status").textContent = "Der Client mit der Nummer "+ whoIsInControlAndLight[1] 
			+ " darf steuern!";
			}
//=============================================Updating Light Information=========================================================
			whiteLight=parseInt(whoIsInControlAndLight[4]);
			redLight=parseInt(whoIsInControlAndLight[5]);
			greenLight=parseInt(whoIsInControlAndLight[6]);
			blueLight=parseInt(whoIsInControlAndLight[7]);
			getLightMessage();

//=============================================Infrared color update for Bebot=======================================================
		if(bebot) {						
			updateBeBot(infraredData);
		}
//=============================================Infrared color update for Amiro==========================================================
		if(!bebot){						
			updateAmiro(infraredData);

		}
            }
		 

		socket_infrared.onclose = function(){
			document.getElementById("WebsocketInfrared").textContent = "Infrarot Websocket: DISCONNECTED";
		}
	} catch(exception) {
		alert('<p>Error' + exception);  
	}




