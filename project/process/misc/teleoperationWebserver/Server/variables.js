/*
 * Author: Markus Gaffke
 * File: variables.js
 * Purpose: setting all global variables
 * Last Changed: 05.07.2014
 *
 */

 //====================================================Funktions & Variables for all Infrared Canvases======================================
var colorcode ="00ff00"; 				//setting the start colorcode for infaredcones to green
var code; 
var isChecked =false; 					//is set to true at first message from Server. Client gets a User_Number and a Flag 								if its running on a Bebot or Amiro
var bebot = false; 					//choosing between Bebot or Amiro (Bebot: bebot=true; // Amiro: bebot=false)
var User_Number, User_Count, total_Number_of_Clients;
var in_control = false; 				//normaly false but for testing purposes set to true
var show_InfraredData = false; 				//for DEBUGGING purposses. If set to true all individual Infrared numbers are shown 								in statusbox 
var controlpannel_free = false; 			//unlocks the controlpannel if the client is in control
var speedbar_old; 					//if speed on speedbar is changed this hold the speed until it is changed again
var speed_cam_send; 					// if true allows the user to send steering commands over cameracanvas
var speedb = document.getElementById("speedbar"); 	//actual speed from speedbar
var mousePos; 						//actual mouse position on canvas
var mousePosDownX, mousePosDownY; 			// saves mouse position when mouse is pressed down for x and y
var speedIMGX, speedIMGY; 				// forward speed, turning speed
var mouseSteeringOn, stop_signal_send; 			
var imgData = document.getElementById("imgnormal");
var whiteLight=0;
var redLight=0;
var greenLight=0;
var blueLight=0;
var textmessageLight;
var leave_get; 

