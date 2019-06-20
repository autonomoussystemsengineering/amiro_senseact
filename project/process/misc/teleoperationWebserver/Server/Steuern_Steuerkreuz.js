/*
 * Author: Markus Gaffke
 * File: Steuern_Steuerkreuz.js
 * Purpose: holding all eventlisteners for control canvas
 * Last Changed: 05.07.2014
 *
 */



var canvas_control = document.getElementById('control');
var context = canvas_control.getContext('2d');

canvas_control.addEventListener('mousemove', function(evt) 
	{
		mousePos = getMousePos(canvas_control, evt);
		if(in_control){document.getElementById("pre_Activ").style.visibility="visible";}
	//document.getElementById("Mouse_Position").textContent = "Mouse Position: " + mousePos.x + ',' + mousePos.y;

	//top left		
	if(mousePos.x < imgData.width/3 && mousePos.y < imgData.height/3) {
		document.getElementById("pre_Activ").style.top="0%";
		document.getElementById("pre_Activ").style.left="-1%";	
		}

	//Click top
	if(mousePos.x > imgData.width/3 && mousePos.x < (imgData.width/3)*2 && mousePos.y < imgData.height/3  ) { 
		document.getElementById("pre_Activ").style.top="0%";
		document.getElementById("pre_Activ").style.left="31.9%";
		} 
	// top right
	if(mousePos.x > (imgData.width/3)*2 && mousePos.y < imgData.height/3) {
		document.getElementById("pre_Activ").style.top="0%";
		document.getElementById("pre_Activ").style.left="66%";
		} 
	//Click mid left
	if(mousePos.x < imgData.width/3 && mousePos.y > imgData.height/3 && mousePos.y < (imgData.height/3)*2) {
		document.getElementById("pre_Activ").style.top="34%";		
		document.getElementById("pre_Activ").style.left="-1%";	
		}
	//Click mid 
	/*
	if(mousePos.x > imgData.width/3 && mousePos.x < (imgData.width/3)*2 
		&& mousePos.y > imgData.height/3 && mousePos.y < (imgData.height/3)*2) {
		document.getElementById("pre_Activ").style.top="34%";		
		document.getElementById("pre_Activ").style.left="32.5%";	
		}	
	*/
	//Click mid right
	if(mousePos.x > (imgData.width/3)*2 && mousePos.y > imgData.height/3 && mousePos.y < (imgData.height/3)*2) {
		document.getElementById("pre_Activ").style.top="34%";		
		document.getElementById("pre_Activ").style.left="66%";
		}
	//Click bottom left
	if(mousePos.x < imgData.width/3 && mousePos.y > (imgData.height/3)*2) { 
		document.getElementById("pre_Activ").style.top="67.5%";		
		document.getElementById("pre_Activ").style.left="-1%"
		}
	//Click bottom 
	if(mousePos.x > imgData.width/3 && mousePos.x < (imgData.width/3)*2 && mousePos.y > (imgData.height/3)*2  ) { 
		document.getElementById("pre_Activ").style.top="67.5%";		
		document.getElementById("pre_Activ").style.left="31.9%"
		}
	//Click bottom right
	if(mousePos.x > (imgData.width/3)*2 && mousePos.y > (imgData.height/3)*2) { 
		document.getElementById("pre_Activ").style.top="67.5%";		
		document.getElementById("pre_Activ").style.left="66%"
		}

	}, false);

canvas_control.addEventListener('mousedown', function(ev) 
	{
		mousePos = getMousePos(canvas_control, ev);
		//document.getElementById("Mouse_Position1").textContent = "Mouse Position: " + mousePos.x + ',' + mousePos.y + "Mouse 			pressed";
		
		if(in_control)
		{	
			//Click top left
			if(mousePos.x < imgData.width/3 && mousePos.y < imgData.height/3) 
			{
				socket_camera.send(1001); 
				document.getElementById("img_OL").style.zIndex='1';
			}
			//Click top
			if(mousePos.x > imgData.width/3 && mousePos.x < (imgData.width/3)*2 && mousePos.y < imgData.height/3  ) 
			{ 
				socket_camera.send(1002); 
				document.getElementById("img_O").style.zIndex='1';
			}
			//Click top right
			if(mousePos.x > (imgData.width/3)*2 && mousePos.y < imgData.height/3) {
				socket_camera.send(1003); 
				document.getElementById("img_OR").style.zIndex='1';
			}
			//Click mid left
			if(mousePos.x < imgData.width/3 && mousePos.y > imgData.height/3 && mousePos.y < (imgData.height/3)*2) 
			{
				socket_camera.send(1004); 
				document.getElementById("img_ML").style.zIndex='1';
			}
			//Click mid right
			if(mousePos.x > (imgData.width/3)*2 && mousePos.y > imgData.height/3 && mousePos.y < (imgData.height/3)*2) 
			{
				socket_camera.send(1005); 
				document.getElementById("img_MR").style.zIndex='1';
			}
			//Click bottom left
			if(mousePos.x < imgData.width/3 && mousePos.y > (imgData.height/3)*2) 
			{ 
				socket_camera.send(1006); 
				document.getElementById("img_UL").style.zIndex='1';
			}
			//Click bottom 
			if(mousePos.x > imgData.width/3 && mousePos.x < (imgData.width/3)*2 && mousePos.y > (imgData.height/3)*2  ) 
			{ 
				socket_camera.send(1007); 
				document.getElementById("img_U").style.zIndex='1';
			}
			//Click bottom right
			if(mousePos.x > (imgData.width/3)*2 && mousePos.y > (imgData.height/3)*2) 
			{ 
				socket_camera.send(1008); 
				document.getElementById("img_UR").style.zIndex='1';
			}
		}
	}, false);




canvas_control.addEventListener('mouseup', function(ev) 
	{
		setControlImgBack();
	}, false);

canvas_control.addEventListener('mouseout', function(ev) 
	{	document.getElementById("pre_Activ").style.visibility="hidden";
		setControlImgBack();
	}, false);




