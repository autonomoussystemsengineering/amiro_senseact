/*
 * Author: Markus Gaffke
 * File: Steuern_Camera.js
 * Purpose: holding all eventlisteners for camera canvas
 * Last Changed: 05.07.2014
 *
 */

var canvas = document.getElementById('image');

canvas.addEventListener('mousemove', function(eff) 
	{
		mousePos = getMousePos(canvas, eff);
		if(mouseSteeringOn && in_control)
		{
			speedIMGX = getSpeed(mousePosDownY - mousePos.y);
			speedIMGY = getSpeed(mousePosDownX - mousePos.x);
			if(mousePos.y < mousePosDownY)
			{		
				if(mousePos.x < mousePosDownX) 
				{
					document.getElementById("Speed_IMG").textContent = "speed forward/sideways: " + speedIMGX+ "/" + speedIMGY;
				}
				if(mousePos.x > mousePosDownX) 
				{
					document.getElementById("Speed_IMG").textContent = "speed forward/sideways: " + speedIMGX + "/" + speedIMGY;
				}
			}else
			{
				if(!stop_signal_send)
				{
					document.getElementById("Speed_IMG").textContent = "speed forward/sideways: STOP" ;
					if(in_control){socket_camera.send(1099);} // send stop Signal
				stop_signal_send = true;				
				}		
			}		
		}      
	}, false);



canvas.addEventListener('mousedown', function(ef) 
	{
		mousePos = getMousePos(canvas, ef);
		mousePosDownX = mousePos.x;
		mousePosDownY = mousePos.y;
		
		mouseSteeringOn = true;
		speed_cam_send = true;
	}, false);

canvas.addEventListener('mouseup', function(ev) 
	{
		mouseSteeringOn = false;
		if(in_control){socket_camera.send(1099);} // send stop Signal
		speed_cam_send = false;
	}, false);

canvas.addEventListener('mouseout', function(ev) 
	{
		mouseSteeringOn = false;
		if(in_control){socket_camera.send(1099);} // send stop Signal
		speed_cam_send = false;
	}, false);
